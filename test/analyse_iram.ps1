# analyse_iram_final_v6.ps1

$mapfile = "C:\Data\Donnees\IOT\WS_Platformio\Plat_Esp_Chaud\.pio\build\upesy_wroom\firmware.map"

Write-Host "Vérification du fichier : $mapfile"
if (-not (Test-Path $mapfile)) { Write-Host "Fichier introuvable"; exit }

# Lire seulement les lignes IRAM
$lines = Get-Content $mapfile | Where-Object { $_ -match '\.iram0\.text|\.iram1\.' }

$iram_symbols = @()

# Regex : adresse hex + taille hex + symbole/fichier
$regex = '0x[0-9a-fA-F]+\s+0x([0-9a-fA-F]+)\s+(.+)'

foreach ($line in $lines) {
    $line = $line.Trim()
    if ($line -match $regex) {
        $size_hex = $matches[1]
        $symbol = $matches[2].Trim()
        $addr_match = $line -match '(0x[0-9a-fA-F]+)\s+0x[0-9a-fA-F]+'

        # On ne prend que les vrais fichiers/fonctions
        if ($symbol -match '\.o|\.cpp|\.c') {
            $size_dec = [Convert]::ToInt32($size_hex,16)
            $address = if ($addr_match) { $matches[1] } else { "N/A" }
            $iram_symbols += [PSCustomObject]@{
                Address=$address
                Size=$size_dec
                Symbol=$symbol
            }
        }
    }
}

# Trier et afficher top 30
$top = $iram_symbols | Sort-Object Size -Descending | Select-Object -First 30
$top | Format-Table @{Label="Address";Expression={$_.Address}}, @{Label="Size (bytes)";Expression={$_.Size}}, @{Label="Symbol/Function";Expression={$_.Symbol}} -AutoSize

# Total IRAM utilisé
$total_bytes = ($iram_symbols | Measure-Object Size -Sum).Sum
$total_kb = [math]::Round($total_bytes / 1024,2)

Write-Host ""
Write-Host "Total IRAM utilisé (tous symboles filtrés) : $total_bytes bytes ($total_kb KB)"
