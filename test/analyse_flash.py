import re
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_FILE = os.path.join(BASE_DIR, "../.pio/build/cam/firmware.map")

TOP_N = 200

def extract_lib_name(path):
    # Cas des librairies : libXXX.a(...)
    if ".a(" in path:
        return path.split(".a(")[0].split("\\")[-1].split("/")[-1] + ".a"
    
    # Cas des fichiers sources (src/)
    return path.split("\\")[-1].split("/")[-1]

#pattern = re.compile(
#    r"\.text\s+0x[0-9a-fA-F]+\s+(0x[0-9a-fA-F]+)\s+(.+)"
#)
pattern = re.compile(
    r"\.text\s+(0x[0-9a-fA-F]+)\s+(0x[0-9a-fA-F]+)\s+(.+)"
)

results = []

print(f"\nTop {TOP_N} des plus gros contributeurs:\n")
total_size = 0

with open(MAP_FILE, "r", encoding="utf-8", errors="ignore") as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if "size before relaxing" in line:
        continue  # ❌ on ignore ces lignes
     
    match = pattern.search(line)
    if match:
        #print(f"{i:12} | {line:<40}")

        addr_hex = match.group(1)
        size_hex = match.group(2)
        location = match.group(3).strip()

        if addr_hex == "0x0000000000000000":
            continue

        size = int(size_hex, 16)
        #print(f"{i:12} | {addr_hex:20} | {size:20} |{location:<40}")
        #print(f"\n")
        # Chercher le nom de la fonction (ligne suivante en général)
        func_name = ""

        # chercher sur les 5 lignes suivantes max
        for j in range(1, 6):
            if i + j >= len(lines):
                break

            next_line = lines[i + j].strip()

            # ignorer lignes inutiles
            if not next_line or "size before relaxing" in next_line:
                continue

            # ligne symbole = commence par adresse
            if next_line.startswith("0x"):
                parts = next_line.split()

                # on prend le dernier élément = nom fonction
                if len(parts) >= 2:
                    func_name = parts[-1]
                    break

        lib_name = extract_lib_name(location)
        results.append((size, func_name, lib_name))
        total_size += size

# Trier par taille décroissante
results.sort(reverse=True, key=lambda x: x[0])

# Afficher top N
print(f"{'Size (bytes)':>12} | {'Function':<40} | File/Lib")
print("-" * 90)

for size, func, loc in results[:TOP_N]:
#for size, func, loc in results:
    print(f"{size:12} | {func:<40} | {loc}")

print("\n" + "-" * 90)
print(f"TOTAL (sections .text utilisées) : {total_size} bytes ({total_size/1024:.1f} Ko)")