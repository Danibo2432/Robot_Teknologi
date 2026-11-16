Feilsøking
Gjør 2.5 på 3 terminaler

Skjekk at ip adresse er 10 på pc og robot

Gjør steg 4 til 8

Trykk på play på robot skjermen etter steg 4.2




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

Bevegelsen er **forutsigbar**, det brukes ingen tilfeldig planlegger (OMPL):

```
nåværende stilling -> hjem -> tegnestilling -> ned til strøket -> tegne -> opp -> tegnestilling -> hjem
```

| Del | Hvordan |
|---|---|
| Til/fra hjem og tegnestilling | Rett linje i leddrommet (alle ledd samtidig), kollisjonssjekket |
| Ned, tegne, opp | Rette linjer for tusjtuppen (kartesisk) |

Med samme mål og samme startstilling blir banen lik hver gang.

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
| UR5e | `192.168.10.101` |
| Lab-PC | `192.168.10.102` (nettmaske `255.255.255.0`) |

```bash
ping -c 3 192.168.10.101
sudo ufw allow 50001:50004/tcp
for p in 29999 30001 30002 30004; do nc -zv -w 2 192.168.10.101 $p; done
```

Ikke slå av brannmuren helt, åpne bare portene over.

> Sjekk med `ip -4 addr` at det kablede kortet (`eno1`) har `192.168.10.102/24`. PC og robot må være på
> samme subnett. Med feil subnett henger driveren på `Initializing driver...` og gir etter ca. 2 min
> `Failed to connect to robot`.

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

### Hjem og tegnestilling

| Stilling | Base, Shoulder, Elbow, Wrist 1, Wrist 2, Wrist 3 (grader) | Rolle |
|---|---|---|
| `--home` | 180,7 / −90 / −90 / −90 / 90 / −180 | Armen rett opp. Start og slutt |
| `--drawing-pose` | 204,2 / −156,6 / −72,5 / −130,9 / 101,6 / −180 | Over arket, tusjen ned, flensen mot basen |

I radianer (brukes i alle kommandoer i seksjon 7 og 8):

```bash
--home "3.154,-1.572,-1.571,-1.571,1.571,-3.142" \
--drawing-pose "3.564,-2.734,-1.265,-2.285,1.774,-3.142"
```

> Arket ligger på **positiv** x og y i `world`. Skriptets innebygde standardverdier er for motsatt
> side, så `--home` og `--drawing-pose` over **må** være med. Uten dem feiler preflight med
> `approach: only X% planned`.

Tegnestillingen bestemmer armformen og tusjens orientering under hele tegningen.
Den er valgt slik at basen bare vrir seg 24° fra hjem, albuen er bøyd, og håndleddet
(Wrist 2 ≈ 100°) er langt unna singularitet.

**Ny tegnestilling** (hvis arket flyttes): stopp programmet, før roboten til ønsket stilling
med pendanten (tusjen rett ned, ca. 15 cm over arket), les av vinklene med
`ros2 topic echo /joint_states --once`, og gi dem i radianer i rekkefølgen
pan, lift, elbow, wrist1, wrist2, wrist3:

```bash
--drawing-pose "3.564,-2.734,-1.265,-2.285,1.774,-3.142"
```

> Programmet avslutter alltid i hjem-stillingen. Står roboten et helt annet sted når
> programmet starter, blir første bevegelse (`to home`) lang. Se på tiden i SUMMARY,
> og la veileder flytte roboten hjem med pendanten hvis den står langt unna.

### Hvis tegningen havner på feil side

**Sjekk (kun planlegging, roboten beveger seg ikke):** start driver og MoveIt (seksjon 4)
og kjør preflight (seksjon 7).

1. Står roboten i RViz i **samme** stilling som den ekte roboten? Hvis **nei**, er det ikke
   retningen som er feil, men modellen eller kalibreringen. Stopp og spør veileder.
2. Ligger den **blå planlagte tegningen** i RViz på samme side av sokkelen som arket
   ligger i virkeligheten? Hvis **ja**, er alt riktig.
3. Ligger den på **motsatt** side, er arkets koordinater målt i pendantens `base`-ramme
   (rotert 180°). Snu da fortegnet på `--center-x/y` **og** bruk tegnestilling og hjem for
   den andre siden (base-leddet 180° mindre):

```bash
--home "0.012,-1.572,-1.571,-1.571,1.571,-3.142" \
--drawing-pose "0.422,-2.734,-1.265,-2.285,1.774,-3.142"
```

Midtpunktet og tegnestillingen må alltid ligge på **samme** side. Står roboten i hjem-stillingen
for den andre siden, blir første bevegelse (`to home`) en 180°-dreining av basen. La veileder
flytte roboten til riktig hjem-stilling med pendanten først.

> Ligger arket et helt annet sted (ikke bare speilet), lær inn en ny tegnestilling over
> arket (se over) og mål midtpunktet med `tf2_echo world tcp`.

### Arkets plassering

Beste metode er å la roboten måle den (gjøres av veileder):

1. Start driveren (seksjon 4).
2. Veileder kjører tusjtuppen forsiktig ned til **arkets midtpunkt** med pendanten.
3. Les av:
   ```bash
   ros2 run tf2_ros tf2_echo world tcp
   ```
4. `Translation: [x, y, z]` gir `--center-x`, `--center-y` og `--center-z` i seksjon 6.

Brukt i labben: senter **(0,45, 0,10, 0,0)** m i `world`.

> **Merk:** pendanten viser posisjoner i rammen `base`, som er rotert 180° i forhold til ROS sin
> `world`/`base_link`. Positiv X og Y på pendanten er negativ X og Y i ROS.

---

## 4. Start systemet

Rekkefølgen er viktig: **driver, så Play på pendanten, så MoveIt.**

### 4.1 Pendanten (forberedelse)

1. Roboten slått på og i **Normal** (grønn).
2. **Installation → URCaps → External Control**: Host IP `192.168.10.102`, port `50002`.
3. Last programmet med **External Control**-noden. **Local mode.**
4. **Ikke trykk Play ennå.**

> **To moduser.** Kommandoene under er forelesningens, med `enable_execution:=false`:
> **kun planlegging**, roboten kan ikke beveges. Det er standard for studenter.
> Skal roboten kjøres (seksjon 8, kun med veileder), startes **både** driver og MoveIt
> på nytt med `enable_execution:=true`.

### 4.2 Terminal 1: driver

```bash
source /opt/ros/humble/setup.bash
source ~/mas248_ws/install/setup.bash
export ROS_DOMAIN_ID=48
ros2 launch mas248_ur5e_real driver.launch.py \
  robot_ip:=192.168.10.101 \
  reverse_ip:=192.168.10.102 \
  calibration:="$HOME/ur5e_calibration.yaml" \
  enable_execution:=true \
  acknowledge_hardware:=yes
```

Vent til `Calibration checked successfully` og `Configured and activated scaled_joint_trajectory_controller`.

### 4.3 Pendanten: Play

**Play → Play from beginning.** Vent på denne linjen i terminal 1:

```
Robot connected to reverse interface. Ready to receive control commands.
```

### 4.4 Terminal 2: MoveIt og RViz

```bash
source /opt/ros/humble/setup.bash
source ~/mas248_ws/install/setup.bash
export ROS_DOMAIN_ID=48
ros2 launch mas248_ur5e_real moveit_real.launch.py \
  robot_ip:=192.168.10.101 \
  reverse_ip:=192.168.10.102 \
  calibration:="$HOME/ur5e_calibration.yaml" \
  enable_execution:=true \
  launch_rviz:=true
```

I RViz:
- **Add → Marker**, Topic `/mas248/planned_drawing` (blå: planlagt tegning)
- **Add → Marker**, Topic `/mas248/tcp_trace` (oransje: det roboten faktisk tegnet)
- Sjekk at roboten i RViz står i **samme stilling** som den ekte roboten.

> Med `enable_execution:=false` virker preflight (seksjon 7), men `--execute` stopper på systemsjekken
> (`MoveIt /execute_trajectory` eller `scaled_joint_trajectory_controller` feiler). Det er meningen.

---

## 5. Sjekker før første bevegelse

Terminal 3:

```bash
ros2 control list_controllers | grep -E "scaled|joint_state"
ros2 topic echo /io_and_status_controller/robot_program_running --once
```

Forventet:

| | `enable_execution:=false` (kun planlegging) | `enable_execution:=true` (kjøring, veileder) |
|---|---|---|
| `joint_state_broadcaster` | `active` | `active` |
| `scaled_joint_trajectory_controller` | `inactive` (riktig, roboten kan ikke beveges) | `active` |
| `robot_program_running` | `data: true` | `data: true` |

Står kontrolleren `inactive` med `enable_execution:=true`, ikke aktiver den manuelt. Ta det med veileder.

`run_picture_real.py` gjør de samme sjekkene automatisk før kjøring.

---

## 6. Lag tegnemål fra et bilde

Bildet legges i `src/mas248_ur5e_demos/images/` (svart strek på hvit bakgrunn, helst PNG).

**Første test** (8 cm, 1 kontur, tusjen 3 cm over arket):

```bash
ros2 run mas248_ur5e_demos image_to_targets \
  ~/mas248_ws/src/mas248_ur5e_demos/images/spiderman.png \
  --width-m 0.08 --center-x 0.45 --center-y 0.10 --center-z 0.0 \
  --tcp-offset-mm 40 --max-contours 8 --max-points 20
```

> **Merk:** `--tcp-offset-mm` er endret fra 30 til 40 mm. Under presentasjonen lå tusjtuppen
> 1 cm for lavt. Det påvirket ikke resultatet, men verdien er rettet.

| Valg | Betydning |
|---|---|
| `--width-m` | Tegningens bredde i **meter** (0.08 = 8 cm, 0.8 = 80 cm) |
| `--center-x/y/z` | Arkets midtpunkt i `world` (seksjon 3) |
| `--tcp-offset-mm` | Tusjens høyde over arket. **40** = tegner i lufta. Ca. **10** = treffer papiret (tuppen ligger 1 cm lavere enn modellen) |
| `--max-contours` | Antall omriss |
| `--max-points` | Punkter per omriss (oppløsning) |

Målene lagres i `/tmp/mas248_targets.json` (forsvinner ved omstart).

---

## 7. Preflight (kun planlegging)

Roboten beveger seg **ikke**.

```bash
PYTHONPATH="$HOME/mas248_ws/src/mas248_ur5e_demos:$PYTHONPATH" \
python3 src/mas248_ur5e_real/scripts/run_picture_real.py \
  --targets /tmp/mas248_targets.json --min-z -0.005 --lift 0.15 \
  --home "3.154,-1.572,-1.571,-1.571,1.571,-3.142" \
  --drawing-pose "3.564,-2.734,-1.265,-2.285,1.774,-3.142"
```

Forventet utskrift:

```
Planning OK
Collisions OK
Trajectory valid
Preflight check passed!
```

Sjekk i RViz at den **blå tegningen** ligger der arket faktisk ligger.
Se også på `to home` i SUMMARY: er den lang (over ca. 15 s), står roboten langt fra hjem.
Sett `--min-z` ut fra målt bordhøyde, så nekter skriptet punkter under bordet.

---

## 8. Kjøring med veileder til stede

**Før:**
- [ ] Veileder er til stede og har godkjent
- [ ] Driver og MoveIt startet på nytt med `enable_execution:=true` (seksjon 4)
- [ ] Arbeidsområdet er tomt, nødstopp innen rekkevidde
- [ ] Preflight passert, blå tegning på riktig sted
- [ ] Hastighetsglideren på pendanten satt til **50 % eller mer** (se seksjon 10 om lavere fart)

**Steg 1: i lufta** (`--tcp-offset-mm 40` fra seksjon 6):

```bash
PYTHONPATH="$HOME/mas248_ws/src/mas248_ur5e_demos:$PYTHONPATH" \
python3 src/mas248_ur5e_real/scripts/run_picture_real.py \
  --targets /tmp/mas248_targets.json --min-z -0.005 --lift 0.15 \
  --home "3.154,-1.572,-1.571,-1.571,1.571,-3.142" \
  --drawing-pose "3.564,-2.734,-1.265,-2.285,1.774,-3.142" \
  --execute --acknowledge-hardware --acknowledge-clearance \
  --no-confirm
```

Trykk Enter for hver bevegelse, `q` + Enter for å stoppe.
For første kjøring kan `--max-strokes 1` legges til, så tegnes bare første strøk.

**Steg 2:** ser alt riktig ut, lag nye mål med lavere `--tcp-offset-mm` (f.eks. 20, så 12, så 10) og kjør igjen. Ved ca. 10 mm treffer tusjen papiret.

**Steg 3:** større bilde (`--width-m 0.18 --max-contours 10 --max-points 150`), først preflight, så kjøring.

Faktisk TCP-bane lagres i `/tmp/mas248_tcp_trace_real.csv` (til resultatdelen i rapporten).

---

## 9. Valg for run_picture_real.py

| Valg | Standard | Betydning |
|---|---|---|
| `--targets` | `/tmp/mas248_l10_targets_lower.json` | Målfil fra `image_to_targets` (bruk `/tmp/mas248_targets.json`) |
| `--home` | armen rett opp, negativ side | Start- og sluttstilling [rad]. **Bruk verdien i seksjon 3** |
| `--drawing-pose` | negativ side | Fast mellomstopp, gir armform og tusjorientering [rad]. **Bruk verdien i seksjon 3** |
| `--lift` | 0.05 | Pen-opp-høyde over tegningen [m]. **Bruk 0.15** i denne cellen |
| `--home-lift` | 0.15 | Løft rett opp etter siste strøk [m] |
| `--z-offset` | 0 | Løfter alle mål [m]. Høyde = `--tcp-offset-mm` + `--z-offset` |
| `--min-z` | ikke satt | Nekter mål under denne høyden [m] |
| `--max-radius` | 0.80 | Nekter mål lenger unna sokkelen [m] |
| `--speed` | **0.5** | Ekstra nedskalering av farten (0 til 1). Høyere verdier ga feil i testene |
| `--max-strokes` | 0 (alle) | Tegn bare de første N strøkene |
| `--no-home` | av | Hopp over hjem, gå rett fra nåværende stilling til tegnestillingen |
| `--execute` | av | Beveg roboten (krever de to under) |
| `--acknowledge-hardware` | av | Bekrefter at dette er ekte robot |
| `--acknowledge-clearance` | av | Bekrefter tomt arbeidsområde og nødstopp |
| `--no-confirm` | av | Ikke spør før hver bevegelse (ikke ved første kjøringer) |

Tusjens orientering hentes automatisk fra `--drawing-pose`. Det finnes ingen
leddrestriksjoner i skriptet; kollisjonssjekken mot modellen (sokkel, bord, roboten selv)
gjelder alltid.

Faktisk fart = MoveIt-skalering (10 %) × `--speed` × hastighetsglideren på pendanten.

---

## 10. Feilsøking

| Symptom | Årsak | Løsning |
|---|---|---|
| Driver henger på `Initializing driver...`, `Failed to connect to robot` | PC og robot på forskjellig subnett, eller feil `robot_ip` | `ip -4 addr` og `ping`, se 2.4 |
| `[FAIL] scaled_joint_trajectory_controller: inactive` | Driver/MoveIt startet med `enable_execution:=false` | Riktig i planleggingsmodus. For kjøring: start på nytt med `true` (seksjon 4) |
| Driver: `launch configuration '...' does not exist` | Kommentar med `$(var ...)` i `config/ur_controllers.yaml` | Fjern `$(var ...)` fra kommentaren, bygg på nytt |
| Driver: `Generated L10 workcell missing` | Pakken ikke konfigurert | Bygg med `--cmake-force-configure` |
| Endringer i modellen vises ikke | Gammel generert beskrivelse | Bygg `mas248_ur5e_real` med `--cmake-force-configure`, start driver og MoveIt på nytt |
| `[FAIL] External Control program running` | Programmet kjører ikke | Play på pendanten **etter** at driveren er startet |
| `[FAIL] scaled_joint_trajectory_controller: no answer` | Controller manager svarte tregt | Kjør på nytt |
| Robot beveger seg ikke | Feil Host IP/port, brannmur | Sjekk External Control (192.168.10.102, 50002) og `ufw` |
| `approach: only X% planned` | Tegnestillingen er på motsatt side av arket, eller trangt nær bord og sokkel | Sjekk at `--home`/`--drawing-pose` og `--center-x/y` er på samme side (seksjon 3). Bruk `--lift 0.15` |
| `straight joint path is in collision` | Veien mellom to stillinger går gjennom sokkel eller bord | Flytt roboten nærmere hjem med pendanten, eller velg en annen `--drawing-pose` |
| `The pen does not point down in --drawing-pose` | Tegnestillingen har feil orientering | Lær inn en ny tegnestilling (seksjon 3) |
| Lange strøk, store håndleddsbevegelser | Tegnestillingen er nær håndledd-singularitet (Wrist 2 nær 0° eller 180°) | Lær inn en tegnestilling med Wrist 2 nær ±90° |
| `to home` tar svært lang tid | Roboten står langt fra hjem, eller et ledd står 360° feil | Flytt roboten hjem med pendanten først |
| `robot is ... rad from the planned start` | Roboten står ikke der planen starter (protective stop, program stoppet) | Sjekk pendanten, Play på nytt |
| `MoveItErrorCodes=-2` i første planlegging | Robot står i ugyldig stilling etter avbrudd | Flytt roboten til en trygg stilling med pendanten (veileder), Play på nytt |
| `Trajectory execution failed error -4` | Kontrolleren avbrøt, ofte protective stop, eller for høy `--speed` | Les meldingen på pendanten, Play på nytt, bruk `--speed 0.5` |
| `error -6` (TIMED_OUT) | Glideren under ca. 50 %: MoveIt tillater bare 2 × planlagt tid + 1 s | Glideren ≥ 50 %, eller øk `allowed_execution_duration_scaling` i `launch/moveit_real.launch.py` (f.eks. 10.0) og start MoveIt på nytt |
| Feil −4/−6 uten grunn, `Moved backwards in time` | Klokkehopp (WSL etter dvale) | Gjelder ikke lab-PC med vanlig Ubuntu. I WSL: `wsl --shutdown` og start på nytt |

### Protective stop og nødstopp

- **Protective stop:** finn årsaken (kollisjon, leddgrense), fjern den, kvitter ut på pendanten, start programmet på nytt, fortsett med lav fart.
- **Nødstopp:** gjør området trygt, frigjør knappen, slå på roboten og slipp bremsene, last External Control-programmet, sjekk arbeidsområdet før ny start.
- **Usikker? Stopp og spør veileder.**

---

## 11. Avslutte

1. Stopp programmet på pendanten.
2. Ctrl + C i terminal 2 (MoveIt), så terminal 1 (driver).
3. Lagre skjermbilder, terminalutskrift og `/tmp/mas248_tcp_trace_real.csv` før PC-en slås av.
