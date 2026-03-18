import re
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MAP_FILE = os.path.join(BASE_DIR, "../.pio/build/cam/firmware.map")

TOP_N = 4

def extract_lib_name(path):
    # Cas des librairies : libXXX.a(...)
    if ".a(" in path:
        return path.split(".a(")[0].split("\\")[-1].split("/")[-1] + ".a"
    
    # Cas des fichiers sources (src/)
    return path.split("\\")[-1].split("/")[-1]

pattern = re.compile(
    r"\.text\s+0x[0-9a-fA-F]+\s+(0x[0-9a-fA-F]+)\s+(.+)"
)

results = []

with open(MAP_FILE, "r", encoding="utf-8", errors="ignore") as f:
    lines = f.readlines()

for i, line in enumerate(lines):
    if "size before relaxing" in line:
        continue  # ❌ on ignore ces lignes
     
    match = pattern.search(line)
    if match:
        size_hex = match.group(1)
        location = match.group(2).strip()

        size = int(size_hex, 16)

        # Chercher le nom de la fonction (ligne suivante en général)
        func_name = ""
        if i + 1 < len(lines):
            next_line = lines[i + 1].strip()
            if next_line.startswith("0x"):
                parts = next_line.split()
                if len(parts) >= 2:
                    func_name = parts[-1]

        lib_name = extract_lib_name(location)
        results.append((size, func_name, lib_name))

# Trier par taille décroissante
results.sort(reverse=True, key=lambda x: x[0])

# Afficher top N
print(f"\nTop {TOP_N} des plus gros contributeurs:\n")
print(f"{'Size (bytes)':>12} | {'Function':<40} | File/Lib")
print("-" * 90)

for size, func, loc in results[:TOP_N]:
    print(f"{size:12} | {func:<40} | {loc}")