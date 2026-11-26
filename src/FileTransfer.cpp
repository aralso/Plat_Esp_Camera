#include <Arduino.h>
#include <ctype.h>
#include <string.h>
#include <strings.h>

#include "variables.h"
#include "SDMMC.h"

namespace {

static const uint8_t kChunkSize = 100;
static const uint8_t kWindowSize = 3;
static const uint8_t kMaxAttempts = 4;
static const uint32_t kAckTimeoutMs = 45000;
static const uint32_t kMinimumSendGapMs = 20;
static const uint32_t kMaxFileSize = 65535UL * kChunkSize;
static const uint8_t kMaxBasename = 63;
static const char kDestination = 'I';

struct TransferAck {
  uint16_t sequence;
  uint8_t code;
};

struct TxChunk {
  bool used;
  bool sent;
  uint8_t status;
  uint8_t attempts;
  uint8_t code;
  uint16_t sequence;
  uint8_t length;
  uint32_t lastSent;
  uint8_t data[kChunkSize];
};

struct RxChunk {
  bool used;
  uint16_t sequence;
  uint8_t code;
  uint8_t length;
  uint8_t data[kChunkSize];
};

static QueueHandle_t txAckQueue = NULL;
static QueueHandle_t rxWakeQueue = NULL;
static SemaphoreHandle_t rxMutex = NULL;
static TaskHandle_t rxWriterTask = NULL;
static bool txRunning = false;
static char txRequestedPath[80] = {};
static portMUX_TYPE txCommitMux = portMUX_INITIALIZER_UNLOCKED;
static uint16_t txFinalSequence = 0;
static bool txCommitReceived = false;

static bool rxActive = false;
static bool rxFinalizing = false;
static uint8_t rxSource = 0;
static uint16_t rxExpectedSequence = 1;
static uint16_t rxFinalSequence = 0;
static uint32_t rxExpectedSize = 0;
static uint32_t rxWrittenSize = 0;
static char rxBasename[kMaxBasename + 1] = {};
static char rxTargetPath[80] = {};
static char rxTemporaryPath[96] = {};
static File rxFile;
static RxChunk rxChunks[kWindowSize] = {};
static bool rxCompleted = false;
static uint8_t rxCompletedSource = 0;
static uint16_t rxCompletedFinalSequence = 0;

static bool safeBasename(const char *name, size_t length)
{
  if ((name == NULL) || (length == 0) || (length > kMaxBasename))
    return false;
  if ((length == 1 && name[0] == '.') ||
      (length == 2 && name[0] == '.' && name[1] == '.'))
    return false;

  for (size_t i = 0; i < length; ++i)
  {
    const unsigned char c = static_cast<unsigned char>(name[i]);
    if (!(isalnum(c) || c == '.' || c == '_' || c == '-'))
      return false;
  }
  return true;
}

static bool captureDateDirectories(const char *basename, char *yearPath,
                                   size_t yearPathSize, char *monthPath,
                                   size_t monthPathSize)
{
  if ((basename == NULL) || (strlen(basename) < 10) ||
      basename[0] != 'C' || basename[2] != '-' || basename[9] != '-' ||
      !isalnum(static_cast<unsigned char>(basename[1])))
    return false;

  for (uint8_t i = 3; i <= 8; ++i)
    if (!isdigit(static_cast<unsigned char>(basename[i])))
      return false;

  const uint8_t month =
      static_cast<uint8_t>((basename[5] - '0') * 10 + basename[6] - '0');
  if (month < 1 || month > 12)
    return false;

  const int yearLength =
      snprintf(yearPath, yearPathSize, "/20%c%c", basename[3], basename[4]);
  const int monthLength =
      snprintf(monthPath, monthPathSize, "%s/%c%c",
               yearPath, basename[5], basename[6]);
  return yearLength > 0 && static_cast<size_t>(yearLength) < yearPathSize &&
         monthLength > 0 && static_cast<size_t>(monthLength) < monthPathSize;
}

static uint32_t transferSegmentCount(uint32_t size)
{
  return size / kChunkSize + (size % kChunkSize != 0);
}

static uint8_t transferAckEmitter()
{
#ifndef END_NODE
  return static_cast<uint8_t>('I' | 0x80);
#elif defined(ESP32_ATTACHED_TO_STM32)
  return UART_ADDRESS_ESP;
#else
  return My_Address & 0x7F;
#endif
}

static void sendTransferReply(uint8_t destination, uint8_t code,
                              uint16_t sequence)
{
  uint8_t message[MSG_SIZE] = {};
  message[0] = static_cast<char>(destination & 0x7F);
  message[1] = 4;
  message[2] = 'Y';
  message[3] = static_cast<char>(code);
  message[4] = static_cast<char>(sequence >> 8);
  message[5] = static_cast<char>(sequence & 0xFF);

  if (envoie_mess_bin_source(message, transferAckEmitter()) != 0)
    Serial.printf("[LORA FILE] Echec envoi %c seq=%u vers %c\n\r",
                  code, sequence, destination);
}

static uint8_t sendTransferPacket(uint8_t destination, uint8_t code,
                                 uint16_t sequence, const uint8_t *body,
                                 uint8_t bodyLength)
{
  const uint8_t payloadLength = static_cast<uint8_t>(4 + bodyLength);
  if ((bodyLength != 0 && body == NULL) ||
      (payloadLength + 2 > MSG_SIZE))
    return 1;

  uint8_t message[MSG_SIZE];
  message[0] = static_cast<char>(destination & 0x7F);
  message[1] = static_cast<char>(payloadLength);
  message[2] = 'Y';
  message[3] = static_cast<char>(code);
  message[4] = static_cast<char>(sequence >> 8);
  message[5] = static_cast<char>(sequence & 0xFF);
  if (bodyLength != 0)
    memcpy(&message[6], body, bodyLength);
  return envoie_mess_bin(message);
}

static bool ensureTxAckQueue()
{
  if (txAckQueue == NULL)
    txAckQueue = xQueueCreate(12, sizeof(TransferAck));
  return txAckQueue != NULL;
}

static bool ensureRxWriter()
{
  if (rxMutex == NULL)
    rxMutex = xSemaphoreCreateMutex();
  if (rxWakeQueue == NULL)
    rxWakeQueue = xQueueCreate(1, sizeof(uint8_t));
  if ((rxMutex == NULL) || (rxWakeQueue == NULL))
    return false;

  if (rxWriterTask == NULL &&
      xTaskCreate([](void *parameter) {
        (void)parameter;
        uint8_t wake = 0;
        for (;;)
        {
          xQueueReceive(rxWakeQueue, &wake, portMAX_DELAY);
          for (;;)
          {
            RxChunk chunk = {};
            bool haveChunk = false;
            bool finalChunk = false;
            uint8_t source = 0;
            uint16_t sequence = 0;

            if (xSemaphoreTake(rxMutex, portMAX_DELAY) != pdTRUE)
              break;
            if (rxActive && !rxFinalizing)
            {
              RxChunk &slot = rxChunks[rxExpectedSequence % kWindowSize];
              if (slot.used && slot.sequence == rxExpectedSequence)
              {
                chunk = slot;
                haveChunk = true;
                source = rxSource;
                sequence = slot.sequence;
              }
            }
            xSemaphoreGive(rxMutex);
            if (!haveChunk)
              break;

            const size_t written = rxFile.write(chunk.data, chunk.length);
            if (written != chunk.length)
            {
              if (xSemaphoreTake(rxMutex, portMAX_DELAY) == pdTRUE)
              {
                rxFinalizing = true;
                source = rxSource;
                xSemaphoreGive(rxMutex);
              }
              rxFile.close();
              SD_MMC.remove(rxTemporaryPath);
              if (xSemaphoreTake(rxMutex, portMAX_DELAY) == pdTRUE)
              {
                rxActive = false;
                rxFinalizing = false;
                for (uint8_t i = 0; i < kWindowSize; ++i)
                  rxChunks[i].used = false;
                xSemaphoreGive(rxMutex);
              }
              sendTransferReply(source, 'E', sequence);
              Serial.printf("[LORA FILE] Erreur ecriture SD seq=%u\n\r", sequence);
              break;
            }

            if (xSemaphoreTake(rxMutex, portMAX_DELAY) != pdTRUE)
              break;
            RxChunk &slot = rxChunks[sequence % kWindowSize];
            slot.used = false;
            rxWrittenSize += chunk.length;
            ++rxExpectedSequence;
            finalChunk = chunk.code == 'C';
            if (finalChunk)
              rxFinalizing = true;
            xSemaphoreGive(rxMutex);

            if (finalChunk)
            {
              rxFile.flush();
              rxFile.close();
              bool finalized = (rxWrittenSize == rxExpectedSize);
              if (finalized && SD_MMC.exists(rxTargetPath) &&
                  !SD_MMC.remove(rxTargetPath))
              {
                Serial.printf("[LORA FILE] Suppression impossible: %s\n\r",
                              rxTargetPath);
                finalized = false;
              }
              if (finalized && !SD_MMC.rename(rxTemporaryPath, rxTargetPath))
              {
                Serial.printf("[LORA FILE] Renommage impossible: %s -> %s\n\r",
                              rxTemporaryPath, rxTargetPath);
                finalized = false;
              }
              if (!finalized)
                SD_MMC.remove(rxTemporaryPath);

              char completedPath[sizeof(rxTargetPath)] = {};
              uint32_t completedSize = 0;
              if (xSemaphoreTake(rxMutex, portMAX_DELAY) == pdTRUE)
              {
                source = rxSource;
                sequence = rxFinalSequence;
                completedSize = rxWrittenSize;
                strncpy(completedPath, rxTargetPath,
                        sizeof(completedPath) - 1);
                rxCompleted = finalized;
                rxCompletedSource = rxSource;
                rxCompletedFinalSequence = rxFinalSequence;
                rxActive = false;
                rxFinalizing = false;
                for (uint8_t i = 0; i < kWindowSize; ++i)
                  rxChunks[i].used = false;
                xSemaphoreGive(rxMutex);
              }

              if (!finalized)
              {
                sendTransferReply(source, 'E', sequence);
                Serial.println("[LORA FILE] Finalisation SD/rename echouee");
                break;
              }

              sendTransferReply(source, 'F', sequence);
              systeme_eve_t event = {};
              event.type = EVENT_RECEPTION_FICHIER;
              event.data = completedSize;
              strncpy(event.path, completedPath, sizeof(event.path) - 1);
              if (eventQueue == NULL ||
                  xQueueSend(eventQueue, &event, portMAX_DELAY) != pdTRUE)
                Serial.println("[LORA FILE] Evenement de reception non emis");
              Serial.printf("[LORA FILE] Recu: %s (%lu octets)\n\r",
                            completedPath, (unsigned long)completedSize);
              break;
            }
          }
        }
      }, "FileRx", 4096, NULL, 1, &rxWriterTask) != pdPASS)
  {
    rxWriterTask = NULL;
    return false;
  }
  return true;
}

static bool sameRxTransfer(uint8_t source, const char *basename,
                           uint32_t size)
{
  return rxActive && source == rxSource && size == rxExpectedSize &&
         strcmp(basename, rxBasename) == 0;
}

static void processTxAcks(TxChunk *chunks, uint8_t &activeCount,
                          uint8_t &acknowledged, bool &failed)
{
  TransferAck ack;
  while (txAckQueue != NULL &&
         xQueueReceive(txAckQueue, &ack, 0) == pdTRUE)
  {
    if (ack.code == 'E')
    {
      failed = true;
      continue;
    }
    for (uint8_t i = 0; i < kWindowSize; ++i)
    {
      TxChunk &chunk = chunks[i];
      if (!chunk.used || chunk.sequence != ack.sequence)
        continue;
      if (ack.code == 'K')
        chunk.status = 1;
      else if (ack.code == 'N' && chunk.status != 1)
        chunk.status = 2;
      break;
    }
  }

  for (uint8_t i = 0; i < kWindowSize; ++i)
  {
    TxChunk &chunk = chunks[i];
    if (!chunk.used)
      continue;
    if (chunk.status == 3)
      failed = true;
    else if (chunk.status == 1)
    {
      chunk.used = false;
      --activeCount;
      ++acknowledged;
    }
  }
}

static bool hasFinalCommit(uint16_t sequence)
{
  portENTER_CRITICAL(&txCommitMux);
  const bool committed =
      txCommitReceived && txFinalSequence == sequence;
  portEXIT_CRITICAL(&txCommitMux);
  return committed;
}

static bool waitForFinalCommit(TxChunk &finalChunk, uint32_t &lastDataSend)
{
  bool retryRequested = false;
  for (;;)
  {
    if (hasFinalCommit(finalChunk.sequence))
      return true;

    TransferAck ack;
    while (txAckQueue != NULL &&
           xQueueReceive(txAckQueue, &ack, 0) == pdTRUE)
    {
      if (ack.code == 'E')
        return false;
      if (ack.sequence == finalChunk.sequence && ack.code == 'N')
        retryRequested = true;
    }

    if (hasFinalCommit(finalChunk.sequence))
      return true;

    const uint32_t now = millis();
    const uint32_t elapsed = now - finalChunk.lastSent;
    if (retryRequested || elapsed >= kAckTimeoutMs)
    {
      if (finalChunk.attempts >= kMaxAttempts)
        return hasFinalCommit(finalChunk.sequence);

      const uint32_t gap = now - lastDataSend;
      if (gap < kMinimumSendGapMs)
        vTaskDelay(pdMS_TO_TICKS(kMinimumSendGapMs - gap));

      sendTransferPacket(kDestination, finalChunk.code,
                         finalChunk.sequence, finalChunk.data,
                         finalChunk.length);
      ++finalChunk.attempts;
      finalChunk.lastSent = millis();
      lastDataSend = finalChunk.lastSent;
      retryRequested = false;
      continue;
    }

    const uint32_t remaining = kAckTimeoutMs - elapsed;
    const uint32_t waitMs = remaining > 250 ? 250 : remaining;
    if (txAckQueue != NULL &&
        xQueueReceive(txAckQueue, &ack, pdMS_TO_TICKS(waitMs)) == pdTRUE)
    {
      if (ack.code == 'E')
        return false;
      if (ack.sequence == finalChunk.sequence && ack.code == 'N')
        retryRequested = true;
    }
  }
}

static bool sendFile(const char *sourcePath)
{
  if ((sourcePath == NULL) || (sourcePath[0] != '/'))
  {
    Serial.println("[LORA FILE] Chemin source invalide");
    return false;
  }

  const char *basename = strrchr(sourcePath, '/');
  basename = (basename == NULL) ? sourcePath : basename + 1;
  const size_t basenameLength = strlen(basename);
  if (!safeBasename(basename, basenameLength))
  {
    Serial.println("[LORA FILE] Nom de fichier invalide");
    return false;
  }
  const char *extension = strrchr(basename, '.');
  if (extension == NULL ||
      (strcasecmp(extension, ".jpg") != 0 &&
       strcasecmp(extension, ".jpeg") != 0))
  {
    Serial.println("[LORA FILE] Le fichier courant n'est pas un JPEG");
    return false;
  }

  File source = SD_MMC.open(sourcePath, FILE_READ);
  if (!source)
  {
    Serial.printf("[LORA FILE] Ouverture impossible: %s\n\r", sourcePath);
    return false;
  }
  const uint32_t fileSize = static_cast<uint32_t>(source.size());
  const uint32_t segmentCount = transferSegmentCount(fileSize);
  if ((fileSize == 0) || (fileSize > kMaxFileSize) ||
      (segmentCount > 0xFFFFUL))
  {
    source.close();
    Serial.printf("[LORA FILE] Taille/sequence non supportee: %lu octets, %lu segments\n\r",
                  (unsigned long)fileSize, (unsigned long)segmentCount);
    return false;
  }

  portENTER_CRITICAL(&txCommitMux);
  txFinalSequence = static_cast<uint16_t>(segmentCount);
  txCommitReceived = false;
  portEXIT_CRITICAL(&txCommitMux);

  uint8_t startBody[5 + kMaxBasename] = {};
  startBody[0] = static_cast<uint8_t>(basenameLength);
  startBody[1] = static_cast<uint8_t>(fileSize >> 24);
  startBody[2] = static_cast<uint8_t>(fileSize >> 16);
  startBody[3] = static_cast<uint8_t>(fileSize >> 8);
  startBody[4] = static_cast<uint8_t>(fileSize);
  memcpy(&startBody[5], basename, basenameLength);
  const uint8_t startBodyLength = static_cast<uint8_t>(5 + basenameLength);

  if (xQueueReset(txAckQueue) != pdPASS)
  {
    source.close();
    Serial.println("[LORA FILE] Impossible de vider la file d'ACK");
    return false;
  }
  sendTransferPacket(kDestination, 'A', 0, startBody, startBodyLength);

  bool startAcknowledged = false;
  uint8_t startAttempts = 1;
  uint32_t lastSend = millis();
  while (!startAcknowledged && startAttempts <= kMaxAttempts)
  {
    const uint32_t started = millis();
    bool retry = false;
    while ((uint32_t)(millis() - started) < kAckTimeoutMs)
    {
      TransferAck ack;
      if (xQueueReceive(txAckQueue, &ack, pdMS_TO_TICKS(100)) != pdTRUE ||
          ack.sequence != 0)
        continue;
      if (ack.code == 'K')
      {
        startAcknowledged = true;
        break;
      }
      if (ack.code == 'E')
      {
        source.close();
        Serial.println("[LORA FILE] Erreur SD distante a l'initialisation");
        return false;
      }
      if (ack.code == 'N')
      {
        retry = true;
        break;
      }
    }
    if (startAcknowledged)
      break;
    if (startAttempts == kMaxAttempts)
      break;
    if (retry || (uint32_t)(millis() - started) >= kAckTimeoutMs)
    {
      const uint32_t elapsed = millis() - lastSend;
      if (elapsed < kMinimumSendGapMs)
        vTaskDelay(pdMS_TO_TICKS(kMinimumSendGapMs - elapsed));
      sendTransferPacket(kDestination, 'A', 0, startBody, startBodyLength);
      ++startAttempts;
      lastSend = millis();
    }
  }
  if (!startAcknowledged)
  {
    source.close();
    Serial.println("[LORA FILE] Aucun ACK pour l'entete");
    return false;
  }

  TxChunk chunks[kWindowSize] = {};
  uint8_t activeCount = 0;
  uint8_t acknowledged = 0;
  uint32_t scheduledBytes = 0;
  uint32_t lastDataSend = 0;
  TxChunk finalChunk = {};
  bool haveFinalChunk = false;
  bool hasDataSend = false;
  bool failed = false;
  uint32_t nextSequence = 1;

  while (!failed && (scheduledBytes < fileSize || activeCount != 0))
  {
    processTxAcks(chunks, activeCount, acknowledged, failed);
    if (failed)
      break;

    if (activeCount == 0)
    {
      for (uint8_t i = 0; i < kWindowSize && scheduledBytes < fileSize; ++i)
      {
        if (nextSequence > 65535UL)
        {
          failed = true;
          break;
        }

        const uint8_t length = static_cast<uint8_t>(
            ((fileSize - scheduledBytes) > kChunkSize)
                ? kChunkSize
                : (fileSize - scheduledBytes));
        const size_t read = source.read(chunks[i].data, length);
        if (read != length)
        {
          failed = true;
          break;
        }
        chunks[i].used = true;
        chunks[i].sent = false;
        chunks[i].status = 0;
        chunks[i].attempts = 0;
        chunks[i].code = (scheduledBytes + length == fileSize) ? 'C' : 'B';
        chunks[i].sequence = static_cast<uint16_t>(nextSequence++);
        chunks[i].length = length;
        chunks[i].lastSent = 0;
        if (chunks[i].code == 'C')
        {
          finalChunk = chunks[i];
          haveFinalChunk = true;
        }
        scheduledBytes += length;
        ++activeCount;
      }
    }
    if (failed)
      break;

    processTxAcks(chunks, activeCount, acknowledged, failed);
    if (failed)
      break;

    const uint32_t now = millis();
    int8_t dueIndex = -1;
    uint32_t waitMs = kAckTimeoutMs;
    for (uint8_t i = 0; i < kWindowSize; ++i)
    {
      TxChunk &chunk = chunks[i];
      if (!chunk.used || chunk.status == 1 || chunk.status == 3)
        continue;

      const bool retryReady = !chunk.sent || chunk.status == 2 ||
          (uint32_t)(now - chunk.lastSent) >= kAckTimeoutMs;
      if (retryReady)
      {
        dueIndex = static_cast<int8_t>(i);
        break;
      }
      const uint32_t remaining =
          kAckTimeoutMs - (uint32_t)(now - chunk.lastSent);
      if (remaining < waitMs)
        waitMs = remaining;
    }

    if (dueIndex >= 0)
    {
      TxChunk &chunk = chunks[static_cast<uint8_t>(dueIndex)];
      if (chunk.attempts >= kMaxAttempts)
      {
        failed = true;
        break;
      }
      if (hasDataSend)
      {
        const uint32_t elapsed = now - lastDataSend;
        if (elapsed < kMinimumSendGapMs)
        {
          TransferAck ack;
          if (xQueueReceive(txAckQueue, &ack,
                            pdMS_TO_TICKS(kMinimumSendGapMs - elapsed)) == pdTRUE)
          {
            if (ack.code == 'E')
            {
              failed = true;
              continue;
            }
            for (uint8_t i = 0; i < kWindowSize; ++i)
              if (chunks[i].used && chunks[i].sequence == ack.sequence)
              {
                chunks[i].status = ack.code == 'K' ? 1 :
                                   ack.code == 'N' && chunks[i].status != 1 ? 2 :
                                   chunks[i].status;
                break;
              }
          }
          continue;
        }
      }

      const uint8_t sendResult =
          sendTransferPacket(kDestination, chunk.code, chunk.sequence,
                             chunk.data, chunk.length);
      ++chunk.attempts;
      chunk.sent = true;
      chunk.status = 0;
      chunk.lastSent = millis();
      lastDataSend = chunk.lastSent;
      hasDataSend = true;
      if (chunk.code == 'C')
        finalChunk = chunk;
      if (sendResult != 0)
        Serial.printf("[LORA FILE] Envoi seq=%u en erreur, retry prevu\n\r",
                      chunk.sequence);
      continue;
    }

    if (activeCount != 0)
    {
      TransferAck ack;
      const uint32_t boundedWait = (waitMs > 250) ? 250 : waitMs;
      if (xQueueReceive(txAckQueue, &ack,
                        pdMS_TO_TICKS(boundedWait)) == pdTRUE)
      {
        if (ack.code == 'E')
          failed = true;
        for (uint8_t i = 0; i < kWindowSize; ++i)
        {
          if (chunks[i].used && chunks[i].sequence == ack.sequence)
          {
            if (ack.code == 'K')
              chunks[i].status = 1;
            else if (ack.code == 'E')
              chunks[i].status = 3;
            else if (ack.code == 'N' && chunks[i].status != 1)
              chunks[i].status = 2;
            break;
          }
        }
      }
    }
  }

  if (!failed)
  {
    if (!haveFinalChunk)
      failed = true;
    else
    {
      finalChunk.lastSent = millis();
      if (!waitForFinalCommit(finalChunk, lastDataSend))
        failed = true;
    }
  }

  source.close();
  if (failed)
    Serial.printf("[LORA FILE] Transfert incomplet (%u segments ACK)\n\r",
                  acknowledged);
  else
    Serial.printf("[LORA FILE] Transfert termine: %s (%lu octets)\n\r",
                  basename, (unsigned long)fileSize);
  return !failed;
}

static void fileSenderTask(void *parameter)
{
  (void)parameter;
  char sourcePath[sizeof(txRequestedPath)];
  memcpy(sourcePath, txRequestedPath, sizeof(sourcePath));
  sourcePath[sizeof(sourcePath) - 1] = '\0';
  if (ensureTxAckQueue())
    sendFile(sourcePath);
  else
    Serial.println("[LORA FILE] File d'ACK indisponible");

  portENTER_CRITICAL(&txCommitMux);
  txRunning = false;
  portEXIT_CRITICAL(&txCommitMux);
  vTaskDelete(NULL);
}

} // namespace

void demarrer_envoi_fichier_lora()
{
  portENTER_CRITICAL(&txCommitMux);
  const bool alreadyRunning = txRunning;
  if (!alreadyRunning)
    txRunning = true;
  portEXIT_CRITICAL(&txCommitMux);
  if (alreadyRunning)
  {
    Serial.println("[LORA FILE] Un transfert est deja en cours");
    return;
  }

  if (!path_c[0])
  {
    portENTER_CRITICAL(&txCommitMux);
    txRunning = false;
    portEXIT_CRITICAL(&txCommitMux);
    Serial.println("[LORA FILE] Aucun chemin JPEG courant");
    return;
  }
  if (!ensureTxAckQueue())
  {
    portENTER_CRITICAL(&txCommitMux);
    txRunning = false;
    portEXIT_CRITICAL(&txCommitMux);
    Serial.println("[LORA FILE] Impossible de creer la file d'ACK");
    return;
  }

  strncpy(txRequestedPath, path_c, sizeof(txRequestedPath) - 1);
  txRequestedPath[sizeof(txRequestedPath) - 1] = '\0';
  if (xTaskCreate(fileSenderTask, "FileTx", 6144, NULL, 2, NULL) != pdPASS)
  {
    portENTER_CRITICAL(&txCommitMux);
    txRunning = false;
    portEXIT_CRITICAL(&txCommitMux);
    Serial.println("[LORA FILE] Impossible de demarrer la tache d'envoi");
  }
}

bool traitement_fichier_lora(St_Message_t *message)
{
  if (message == NULL || message->longueur < 4)
    return false;

  uint8_t *bytes = reinterpret_cast<uint8_t *>(message->msg);
  const uint16_t wireLength = message->longueur - 1;
  if ((bytes[0] & 0x80) == 0 || wireLength < 4 || bytes[2] != 'Y')
    return false;

  const uint8_t code = bytes[3];
  if (code != 'A' && code != 'B' && code != 'C' &&
      code != 'K' && code != 'N' && code != 'E' && code != 'F')
    return false;

  const uint8_t source = bytes[1] & 0x7F;
  const uint8_t payloadLength = static_cast<uint8_t>(wireLength - 2);
  if (payloadLength < 4)
  {
    sendTransferReply(source, 'N', 0);
    return true;
  }
  const uint16_t sequence =
      (static_cast<uint16_t>(bytes[4]) << 8) | bytes[5];

  if (code == 'F')
  {
    portENTER_CRITICAL(&txCommitMux);
    if (txRunning && sequence == txFinalSequence)
      txCommitReceived = true;
    portEXIT_CRITICAL(&txCommitMux);
    return true;
  }

  if (code == 'K' || code == 'N' || code == 'E')
  {
    if (txAckQueue != NULL)
    {
      TransferAck ack = { sequence, code };
      xQueueSend(txAckQueue, &ack, 0);
    }
    return true;
  }

  if (code == 'A')
  {
    if (sequence != 0 || payloadLength < 9)
    {
      sendTransferReply(source, 'N', sequence);
      return true;
    }

    const uint8_t nameLength = bytes[6];
    const uint32_t fileSize =
        (static_cast<uint32_t>(bytes[7]) << 24) |
        (static_cast<uint32_t>(bytes[8]) << 16) |
        (static_cast<uint32_t>(bytes[9]) << 8) |
        static_cast<uint32_t>(bytes[10]);
    const uint32_t segmentCount = transferSegmentCount(fileSize);
    const uint8_t expectedPayloadLength =
        static_cast<uint8_t>(9 + nameLength);
    if ((payloadLength != expectedPayloadLength) ||
        (wireLength != static_cast<uint16_t>(2 + expectedPayloadLength)) ||
        !safeBasename(reinterpret_cast<const char *>(&bytes[11]), nameLength) ||
        fileSize == 0 || fileSize > kMaxFileSize ||
        segmentCount > 0xFFFFUL)
    {
      sendTransferReply(source, 'N', sequence);
      return true;
    }

    char basename[kMaxBasename + 1] = {};
    memcpy(basename, &bytes[11], nameLength);
    char yearPath[8] = {};
    char monthPath[16] = {};
    if (!captureDateDirectories(basename, yearPath, sizeof(yearPath),
                                monthPath, sizeof(monthPath)))
    {
      sendTransferReply(source, 'N', sequence);
      return true;
    }
    if (!ensureRxWriter())
    {
      sendTransferReply(source, 'N', sequence);
      return true;
    }

    if (xSemaphoreTake(rxMutex, portMAX_DELAY) != pdTRUE)
    {
      sendTransferReply(source, 'N', sequence);
      return true;
    }
    if (sameRxTransfer(source, basename, fileSize))
    {
      xSemaphoreGive(rxMutex);
      sendTransferReply(source, 'K', sequence);
      return true;
    }
    if (rxActive)
    {
      xSemaphoreGive(rxMutex);
      sendTransferReply(source, 'N', sequence);
      return true;
    }

    if (!SD_MMC.exists(yearPath) && !SD_MMC.mkdir(yearPath))
    {
      Serial.printf("[LORA FILE] Creation du repertoire impossible: %s\n\r",
                    yearPath);
      xSemaphoreGive(rxMutex);
      sendTransferReply(source, 'E', sequence);
      return true;
    }
    if (!SD_MMC.exists(monthPath) && !SD_MMC.mkdir(monthPath))
    {
      Serial.printf("[LORA FILE] Creation du repertoire impossible: %s\n\r",
                    monthPath);
      xSemaphoreGive(rxMutex);
      sendTransferReply(source, 'E', sequence);
      return true;
    }
    const int targetLength =
        snprintf(rxTargetPath, sizeof(rxTargetPath), "%s/%s",
                 monthPath, basename);
    const int temporaryLength =
        snprintf(rxTemporaryPath, sizeof(rxTemporaryPath), "%s/.%s.part",
                 monthPath, basename);
    if (targetLength <= 0 ||
        static_cast<size_t>(targetLength) >= sizeof(rxTargetPath) ||
        temporaryLength <= 0 ||
        static_cast<size_t>(temporaryLength) >= sizeof(rxTemporaryPath))
    {
      xSemaphoreGive(rxMutex);
      sendTransferReply(source, 'N', sequence);
      return true;
    }
    if (SD_MMC.exists(rxTemporaryPath) && !SD_MMC.remove(rxTemporaryPath))
    {
      Serial.printf("[LORA FILE] Suppression du temporaire impossible: %s\n\r",
                    rxTemporaryPath);
      xSemaphoreGive(rxMutex);
      sendTransferReply(source, 'E', sequence);
      return true;
    }
    rxFile = SD_MMC.open(rxTemporaryPath, FILE_WRITE);
    if (!rxFile)
    {
      Serial.printf("[LORA FILE] Ouverture impossible: %s\n\r",
                    rxTemporaryPath);
      xSemaphoreGive(rxMutex);
      sendTransferReply(source, 'E', sequence);
      return true;
    }

    rxSource = source;
    rxExpectedSequence = 1;
    rxExpectedSize = fileSize;
    rxWrittenSize = 0;
    rxFinalSequence = 0;
    rxFinalizing = false;
    rxActive = true;
    rxCompleted = false;
    memcpy(rxBasename, basename, sizeof(rxBasename));
    for (uint8_t i = 0; i < kWindowSize; ++i)
      rxChunks[i].used = false;
    xSemaphoreGive(rxMutex);
    sendTransferReply(source, 'K', sequence);
    return true;
  }

  if (!ensureRxWriter())
  {
    sendTransferReply(source, 'N', sequence);
    return true;
  }
  if (xSemaphoreTake(rxMutex, portMAX_DELAY) != pdTRUE)
  {
    sendTransferReply(source, 'N', sequence);
    return true;
  }

  if (!rxActive)
  {
    const bool duplicateFinal = rxCompleted &&
        source == rxCompletedSource && sequence == rxCompletedFinalSequence &&
        code == 'C';
    xSemaphoreGive(rxMutex);
    sendTransferReply(source, duplicateFinal ? 'F' : 'N', sequence);
    return true;
  }
  if (source != rxSource || sequence == 0 ||
      payloadLength <= 4 || payloadLength > 4 + kChunkSize)
  {
    xSemaphoreGive(rxMutex);
    sendTransferReply(source, 'N', sequence);
    return true;
  }

  const uint8_t dataLength = static_cast<uint8_t>(payloadLength - 4);
  const uint32_t expectedOffset =
      static_cast<uint32_t>(sequence - 1) * kChunkSize;
  const uint32_t segmentCount = transferSegmentCount(rxExpectedSize);
  const bool lastChunk = code == 'C';
  if (sequence > segmentCount ||
      (!lastChunk && (dataLength != kChunkSize ||
                      expectedOffset + dataLength >= rxExpectedSize)) ||
      expectedOffset + dataLength > rxExpectedSize ||
      (lastChunk && expectedOffset + dataLength != rxExpectedSize) ||
      (rxFinalSequence != 0 && sequence > rxFinalSequence) ||
      (lastChunk && rxFinalSequence != 0 && sequence != rxFinalSequence))
  {
    xSemaphoreGive(rxMutex);
    sendTransferReply(source, 'N', sequence);
    return true;
  }

  if (sequence < rxExpectedSequence)
  {
    xSemaphoreGive(rxMutex);
    sendTransferReply(source, 'K', sequence);
    return true;
  }
  if (sequence >= static_cast<uint32_t>(rxExpectedSequence) + kWindowSize)
  {
    xSemaphoreGive(rxMutex);
    sendTransferReply(source, 'N', sequence);
    return true;
  }

  RxChunk &slot = rxChunks[sequence % kWindowSize];
  if (slot.used)
  {
    const bool duplicate = slot.sequence == sequence &&
        slot.code == code && slot.length == dataLength &&
        memcmp(slot.data, &bytes[6], dataLength) == 0;
    xSemaphoreGive(rxMutex);
    sendTransferReply(source, duplicate ? 'K' : 'N', sequence);
    return true;
  }
  if (rxFinalizing)
  {
    xSemaphoreGive(rxMutex);
    sendTransferReply(source, 'N', sequence);
    return true;
  }
  if (lastChunk)
    rxFinalSequence = sequence;
  slot.used = true;
  slot.sequence = sequence;
  slot.code = code;
  slot.length = dataLength;
  memcpy(slot.data, &bytes[6], dataLength);
  xSemaphoreGive(rxMutex);

  uint8_t wake = 1;
  xQueueSend(rxWakeQueue, &wake, 0);
  sendTransferReply(source, 'K', sequence);
  return true;
}

void traiter_reception_fichier_lora_event(const char *finalPath)
{
  Serial.printf("[LORA FILE] Evenement: fichier recu et finalise: %s\n\r",
                (finalPath != NULL && finalPath[0] != '\0')
                    ? finalPath
                    : "(chemin indisponible)");
}
