# MAS248: Tegning med ekte UR5e (lab-PC)

Manual for å kjøre tegneprogrammet på lab-PC-en som er koblet til robotarmen.
Alt er testet mot URSim (UR sin offisielle simulator) med samme driver, kontroller og kode.

> **Sikkerhet først**
> - Roboten skal **aldri** beveges uten at foreleser eller studentassistent er til stede.
> - Studenter kjører kun **preflight** (planlegging). `--execute` brukes bare med veileder.
> - Én person står ved **nødstoppen** på pendanten under hele kjøringen.
> - Ctrl + C i terminalen stopper **programmet**, ikke en bevegelse som allerede kjører. Bruk pendanten for å stoppe roboten.

---

## Innhold

1. [Oversikt](#1-oversikt)
2. [Engangsoppsett på lab-PC-en](#2-engangsoppsett-på-lab-pc-en)
3. [Mål cellen og oppdater modellen](#3-mål-cellen-og-oppdater-modellen)
4. [Start systemet](#4-start-systemet)
5. [Sjekker før første bevegelse](#5-sjekker-før-første-bevegelse)
6. [Lag tegnemål fra et bilde](#6-lag-tegnemål-fra-et-bilde)
7. [Preflight (kun planlegging)](#7-preflight-kun-planlegging)
8. [Kjøring med veileder til stede](#8-kjøring-med-veileder-til-stede)
9. [Valg for run_picture_real.py](#9-valg-for-run_picture_realpy)
10. [Feilsøking](#10-feilsøking)
11. [Avslutte](#11-avslutte)

---

## 1. Oversikt

| Pakke | Hva den gjør |
|---|---|
| `mas248_ur5e` | Workcell-modellen (UR5e, sokkel, spindel, TCP i tusjtuppen, bord) |
| `mas248_ur5e_moveit_config` | MoveIt-oppsett (planleggingsgruppe `manipulator`, SRDF, kinematikk) |
| `mas248_ur5e_demos` | `image_to_targets` (bilde til mål) og `run_picture` (simulering) |
| `mas248_ur5e_real` | Driver og MoveIt for ekte robot, og `run_picture_real.py` |

Flyten:

```
bilde -> image_to_targets -> /tmp/mas248_targets.json -> run_picture_real.py -> MoveIt -> UR-driver -> UR5e
```

`run_picture_real.py` planlegger **hele** tegningen først. Feiler én del, kjøres ingenting.

---

## 2. Engangsoppsett på lab-PC-en

### 2.1 Programvare

Ubuntu 22.04 og ROS 2 Humble. Sjekk at pakkene finnes:

```bash
sudo apt install -y ros-humble-ur ros-humble-moveit ros-humble-ros2-control ros-humble-ros2-controllers python3-opencv
```

### 2.2 Hent koden

Repoet har pakkene i `src/`, så det kan klones direkte som workspace:

```bash
git clone <repo-url> ~/mas248_ws
cd ~/mas248_ws && git checkout Lenin
cp ~/mas248_ws/ur5e_calibration.yaml ~/ur5e_calibration.yaml
```

### 2.3 Bygg

```bash
cd ~/mas248_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> **Viktig:** `mas248_ur5e_real` lager sin robotbeskrivelse fra workcell-filen bare når pakken
> *konfigureres*. Etter **enhver** endring i `ur5e_workcell.urdf.xacro` må den bygges slik:
> ```bash
> colcon build --symlink-install --packages-select mas248_ur5e_real --cmake-force-configure
> ```

### 2.4 Nettverk

| | IP |
|---|---|
| UR5e | `192.168.1.101` |
| Lab-PC | `192.168.1.102` (nettmaske `255.255.255.0`) |

```bash
ping -c 3 192.168.1.101
sudo ufw allow 50001:50004/tcp
for p in 29999 30001 30002 30004; do nc -zv -w 2 192.168.1.101 $p; done
```

Ikke slå av brannmuren helt, åpne bare portene over.

### 2.5 I hver ny terminal

```bash
source /opt/ros/humble/setup.bash
source ~/mas248_ws/install/setup.bash
export ROS_DOMAIN_ID=48
cd ~/mas248_ws
```

---

## 3. Mål cellen og oppdater modellen

Modellen må stemme med den ekte cellen. Feil mål gir feil tusjhøyde.

| Mål | Nå i modellen | Hvor |
|---|---|---|
| Sokkelhøyde (bord til robotfot) | 0,31 m | `pedestal_link` og `ur_robot` origin i `mas248_ur5e/urdf/ur5e_workcell.urdf.xacro` |
| Tusjtupp fra flensen | 0,10 m ned, 0,065 m langs `tool0` +Y | joint `spindle_to_tcp` |
| Bordflate | z = 0 (sokkelen står på bordet) | `table_link` |

Etter endringer: bygg med `--cmake-force-configure` (se 2.3).

### Arkets plassering

Beste metode er å la roboten måle den (gjøres av veileder):

1. Start driveren (seksjon 4).
2. Veileder kjører tusjtuppen forsiktig ned til **arkets midtpunkt** med pendanten.
3. Les av:
   ```bash
   ros2 run tf2_ros tf2_echo world tcp
   ```
4. `Translation: [x, y, z]` gir `--center-x`, `--center-y` og `--center-z` i seksjon 6.

Foreløpig estimat: senter **(−0,45, −0,10, 0,0)** m i `world`.

> **Merk:** pendanten viser posisjoner i rammen `base`, som er rotert 180° i forhold til ROS sin
> `world`/`base_link`. Positiv X og Y på pendanten er negativ X og Y i ROS.

---

## 4. Start systemet

Rekkefølgen er viktig: **driver, så Play på pendanten, så MoveIt.**

### 4.1 Pendanten (forberedelse)

1. Roboten slått på og i **Normal** (grønn).
2. **Installation → URCaps → External Control**: Host IP `192.168.1.102`, port `50002`.
3. Last programmet med **External Control**-noden. **Local mode.**
4. **Ikke trykk Play ennå.**

### 4.2 Terminal 1: driver

```bash
ros2 launch mas248_ur5e_real driver.launch.py \
  robot_ip:=192.168.1.101 reverse_ip:=192.168.1.102 \
  calibration:="$HOME/ur5e_calibration.yaml" \
  enable_execution:=true acknowledge_hardware:=yes
```

Vent til `Calibration checked successfully` og `Configured and activated scaled_joint_trajectory_controller`.

### 4.3 Pendanten: Play

**Play → Play from beginning.** Vent på denne linjen i terminal 1:

```
Robot connected to reverse interface. Ready to receive control commands.
```

### 4.4 Terminal 2: MoveIt og RViz

```bash
ros2 launch mas248_ur5e_real moveit_real.launch.py \
  robot_ip:=192.168.1.101 reverse_ip:=192.168.1.102 \
  calibration:="$HOME/ur5e_calibration.yaml" enable_execution:=true
```

I RViz:
- **Add → Marker**, Topic `/mas248/planned_drawing` (blå: planlagt tegning)
- **Add → Marker**, Topic `/mas248/tcp_trace` (oransje: det roboten faktisk tegnet)
- Sjekk at roboten i RViz står i **samme stilling** som den ekte roboten.

> Kun planlegging uten mulighet for bevegelse: start både driver og MoveIt med `enable_execution:=false`.

---

## 5. Sjekker før første bevegelse

Terminal 3:

```bash
ros2 control list_controllers | grep -E "scaled|joint_state"
ros2 topic echo /io_and_status_controller/robot_program_running --once
```

Forventet:
- `scaled_joint_trajectory_controller ... active`
- `joint_state_broadcaster ... active`
- `data: true`

`run_picture_real.py` gjør de samme sjekkene automatisk før kjøring.

---

## 6. Lag tegnemål fra et bilde

Bildet legges i `src/mas248_ur5e_demos/images/` (svart strek på hvit bakgrunn, helst PNG).

**Første test** (lite, 4 cm, 1 kontur, tusjen 3 cm over arket):

```bash
ros2 run mas248_ur5e_demos image_to_targets \
  ~/mas248_ws/src/mas248_ur5e_demos/images/pumpkin_outline.jpg \
  --width-m 0.04 --center-x -0.45 --center-y -0.10 --center-z 0.0 \
  --tcp-offset-mm 30 --max-contours 1 --max-points 20
```

| Valg | Betydning |
|---|---|
| `--width-m` | Tegningens bredde i meter |
| `--center-x/y/z` | Arkets midtpunkt i `world` (seksjon 3) |
| `--tcp-offset-mm` | Tusjens høyde over arket. **30** = tegner i lufta. **0** = treffer papiret |
| `--max-contours` | Antall omriss |
| `--max-points` | Punkter per omriss (oppløsning) |

Målene lagres i `/tmp/mas248_targets.json` (forsvinner ved omstart).

---

## 7. Preflight (kun planlegging)

Roboten beveger seg **ikke**.

```bash
PYTHONPATH="$HOME/mas248_ws/src/mas248_ur5e_demos:$PYTHONPATH" \
python3 src/mas248_ur5e_real/scripts/run_picture_real.py \
  --targets /tmp/mas248_targets.json --min-z -0.005 --lift 0.15
```

Forventet utskrift:

```
Planning OK
Collisions OK
Trajectory valid
Preflight check passed!
```

Sjekk i RViz at den **blå tegningen** ligger der arket faktisk ligger.
Sett `--min-z` ut fra målt bordhøyde, så nekter skriptet punkter under bordet.

---

## 8. Kjøring med veileder til stede

**Før:**
- [ ] Veileder er til stede og har godkjent
- [ ] Arbeidsområdet er tomt, nødstopp innen rekkevidde
- [ ] Preflight passert, blå tegning på riktig sted
- [ ] Hastighetsglideren på pendanten satt lavt (10 til 20 %)

**Steg 1: i lufta, med bekreftelse før hver bevegelse** (`--tcp-offset-mm 30` fra seksjon 6):

```bash
PYTHONPATH="$HOME/mas248_ws/src/mas248_ur5e_demos:$PYTHONPATH" \
python3 src/mas248_ur5e_real/scripts/run_picture_real.py \
  --targets /tmp/mas248_targets.json --min-z -0.005 --lift 0.15 --speed 0.5 \
  --execute --acknowledge-hardware --acknowledge-clearance
```

Trykk Enter for hver bevegelse, `q` + Enter for å stoppe.

**Steg 2:** ser alt riktig ut, lag nye mål med lavere `--tcp-offset-mm` (f.eks. 5, så 1, så 0) og kjør igjen.

**Steg 3:** større bilde (`--width-m 0.18 --max-contours 10 --max-points 150`), først preflight, så kjøring.

Faktisk TCP-bane lagres i `/tmp/mas248_tcp_trace_real.csv` (til resultatdelen i rapporten).

---

## 9. Valg for run_picture_real.py

| Valg | Standard | Betydning |
|---|---|---|
| `--targets` | `/tmp/mas248_l10_targets_lower.json` | Målfil fra `image_to_targets` |
| `--lift` | 0.05 | Pen-opp-høyde over tegningen [m]. **Bruk 0.15** i denne cellen |
| `--home-lift` | 0.15 | Løft før planlegging hjem [m] |
| `--z-offset` | 0 | Løfter alle mål [m] |
| `--tool-yaw-deg` | −90 | Roterer tusjen rundt egen akse, holder håndleddet unna singularitet |
| `--min-z` | ikke satt | Nekter mål under denne høyden [m] |
| `--max-radius` | 0.80 | Nekter mål lenger unna sokkelen [m] |
| `--speed` | 0.5 | Ekstra nedskalering av farten (0 til 1) |
| `--max-strokes` | 0 (alle) | Tegn bare de første N strøkene |
| `--no-home` | av | Start fra nåværende stilling i stedet for hjem |
| `--execute` | av | Beveg roboten (krever de to under) |
| `--acknowledge-hardware` | av | Bekrefter at dette er ekte robot |
| `--acknowledge-clearance` | av | Bekrefter tomt arbeidsområde og nødstopp |
| `--no-confirm` | av | Ikke spør før hver bevegelse (ikke ved første kjøringer) |

---

## 10. Feilsøking

| Symptom | Årsak | Løsning |
|---|---|---|
| Driver: `launch configuration '...' does not exist` | Kommentar med `$(var ...)` i `config/ur_controllers.yaml` | Fjern `$(var ...)` fra kommentaren, bygg på nytt |
| Driver: `Generated L10 workcell missing` | Pakken ikke konfigurert | Bygg med `--cmake-force-configure` |
| Endringer i modellen vises ikke | Gammel generert beskrivelse | Bygg `mas248_ur5e_real` med `--cmake-force-configure`, start driver og MoveIt på nytt |
| `[FAIL] External Control program running` | Programmet kjører ikke | Play på pendanten **etter** at driveren er startet |
| `[FAIL] scaled_joint_trajectory_controller: no answer` | Controller manager svarte tregt | Kjør på nytt |
| Robot beveger seg ikke | Feil Host IP/port, brannmur | Sjekk External Control (192.168.1.102, 50002) og `ufw` |
| `approach: only X% planned` / `MoveItErrorCodes=99999` på approach | Trangt område nær bord og sokkel | Bruk `--lift 0.15` (eller høyere) |
| `No IK solution near home` | Punktet kan ikke nås med tusjen ned | Sjekk `--center-x/y` (fortegn, se seksjon 3) |
| Svært store håndleddsbevegelser, lang strøktid | Håndledd-singularitet | `--tool-yaw-deg -90` (standard) eller `90` |
| `MoveItErrorCodes=-2` i første planlegging | Robot står i ugyldig stilling etter avbrudd | Flytt roboten til en trygg stilling med pendanten (veileder), Play på nytt |
| `Trajectory execution failed error -4` | Kontrolleren avbrøt, ofte protective stop | Les meldingen på pendanten, fjern årsaken, Play på nytt, start lavt |
| `error -6` (TIMED_OUT) | Bevegelsen tok lenger tid enn ventet | Sjekk pendanten, kjør på nytt med lavere fart |

### Protective stop og nødstopp

- **Protective stop:** finn årsaken (kollisjon, leddgrense), fjern den, kvitter ut på pendanten, start programmet på nytt, fortsett med lav fart.
- **Nødstopp:** gjør området trygt, frigjør knappen, slå på roboten og slipp bremsene, last External Control-programmet, sjekk arbeidsområdet før ny start.
- **Usikker? Stopp og spør veileder.**

---

## 11. Avslutte

1. Stopp programmet på pendanten.
2. Ctrl + C i terminal 2 (MoveIt), så terminal 1 (driver).
3. Lagre skjermbilder, terminalutskrift og `/tmp/mas248_tcp_trace_real.csv` før PC-en slås av.
