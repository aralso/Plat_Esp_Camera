# analyse_rodata_flexible.py
MAP_FILE = r"C:\Data\Donnees\IOT\WS_Platformio\Plat_Esp_Camera\.pio\build\cam\firmware.map"

rodata_entries = []
total_size = 0

with open(MAP_FILE, "r", encoding="utf-8", errors="ignore") as f:
    lines = [line.rstrip() for line in f]

i = 0
while i < len(lines):
    line = lines[i]

    # on ne s'intéresse qu'aux lignes contenant .rodata
    if ".text" in line.lower():
        parts = line.split()

        # si la ligne contient déjà au moins 3 colonnes numériques (adresse + taille + lib/objet)
        if len(parts) >= 3 and (parts[1].startswith("0x") or parts[1].isdigit()):
            # adresse = parts[1] (on ignore), taille = parts[2]
            adresse = parts[1]
            size_str = parts[2]
            location = " ".join(parts[3:]) if len(parts) > 3 else ""
        else:
            # sinon on prend la ligne suivante
            i += 1
            if i >= len(lines):
                break
            next_line = lines[i]
            parts = next_line.split()
            if len(parts) < 3:
                i += 1
                continue
            size_str = parts[1]
            adresse = parts[0]
            try:
                sizeL = int(size_str, 16) if size_str.startswith("0x") else int(size_str)
            except ValueError:
                sizeL = 0
            location = " ".join(parts[2:]) if len(parts) > 2 else ""
            #if (sizeL>24000):
            #print(f"L2:{sizeL} | {adresse} | {location} | {next_line:40}")

        # convertir la taille en entier
        try:
            size = int(size_str, 16) if size_str.startswith("0x") else int(size_str)
        except ValueError:
            i += 1
            continue

        # extraire lib et objet
        if "(" in location:
            lib = location.split("(")[0].split("\\")[-1]
            obj = location.split("(")[-1].replace(")", "")
        else:
            lib = location.split("\\")[-1] if "\\" in location else location
            obj = ""

        if adresse != "0x0000000000000000" and size < 1000000:
            #print(f"Size: {size} | {adresse} | {location} | {line:40}")
            rodata_entries.append((size, obj, lib, adresse))
            total_size += size

    i += 1

# trier par taille décroissante
rodata_entries.sort(key=lambda x: x[0], reverse=True)

# afficher top 40
print(f"Total .rodata = {total_size} bytes ({total_size/1024:.1f} KB)\n")
print(f"{'Size (bytes)':>12} | {'Object':30} | {'Library'}")
print("-"*70)
for size, obj, lib, adresse in rodata_entries[:600]:
    print(f"{size:12} | {obj:30} | {adresse} | {lib}")

# calcul du total par librairie
lib_totals = {}

for size, obj, lib, adresse in rodata_entries:
    if lib not in lib_totals:
        lib_totals[lib] = 0
    lib_totals[lib] += size

# tri par taille décroissante
sorted_libs = sorted(lib_totals.items(), key=lambda x: x[1], reverse=True)

print("\nTotal par librairie :\n")
print(f"{'Size (bytes)':>12} | {'Library'}")
print("-"*40)

for lib, total in sorted_libs:
    print(f"{total:12} | {lib}")