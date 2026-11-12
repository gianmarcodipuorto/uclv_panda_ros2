<p align="right">
  <a href="ARCHITECTURE.md"><img src="https://img.shields.io/badge/English-8c959f?style=for-the-badge" alt="English"></a>
  <a href="ARCHITECTURE.it.md"><img src="https://img.shields.io/badge/Italiano-0969da?style=for-the-badge" alt="Italiano"></a>
</p>

# Come funzionano i bridge

Questa pagina è per chi modifica il codice. Per usare il driver basta il [README](../README.it.md).

## Il percorso di un comando

```
client ──topic──▶ subscription ──coda lock-free──▶ loop 1 kHz (thread di libfranka) ──▶ robot
                                                     │  jitter buffer: riproduce i campioni
                                                     │    secondo header.stamp
                                                     │  tracker: rende il comando accettabile
                                                     │    per il robot
/joint_states, /panda/robot_state ◀── publisher ◀────┘  stato del loop, senza lock
```

Ogni bridge usa questi thread:

- **Thread robot.** Con il bridge attivo esegue `robot.control()`, che libfranka porta a SCHED_FIFO 99. Con il bridge inattivo legge lo stato con `readOnce()` a `publish_freq`.
- **Thread publisher.** Pubblica stato e status a `publish_freq` e scrive nei log gli eventi del loop.
- **Executor.** Un MultiThreadedExecutor con 5 thread: lifecycle, comandi, azioni e servizi, recovery, pinza.
- **Thread della pinza.** Con una Franka Hand: uno ne legge lo stato e lo pubblica, uno esegue i suoi comandi (vedi [La Franka Hand](#la-franka-hand)).

Nel callback a 1 kHz non ci sono allocazioni, lock, log o chiamate ROS. Comandi e stato passano per code lock-free e box non bloccanti.

## Macchina a stati e tracker

Tutti i bridge condividono la stessa macchina a stati (holding, moving, stopping, finished), in `pipeline.hpp/.cpp`. Cambia solo il **tracker**, cioè come il riferimento diventa un comando accettabile per il robot:

| Bridge | Tracker | Cosa fa |
|---|---|---|
| posizioni | `JointTracker` | controlla velocità, accelerazione e jerk di ogni comando come fa il robot (differenze finite); se non passa, converge al riferimento con Ruckig |
| velocità | `JointVelocityTracker` | lo stesso controllo, un ordine più in basso; integra le velocità come il robot e frena prima dei finecorsa |
| coppie | `JointTorqueTracker` | satura ai limiti di coppia e limita la derivata (il robot rifiuta le discontinuità); nei moti internal è un'impedenza di giunto |
| posa cartesiana | `CartesianPoseTracker` | il robot ricava il twist da due pose consecutive e controlla le norme di velocità, accelerazione e jerk di traslazione, rotazione e gomito: il tracker fa lo stesso; se non passa, converge con Ruckig |
| velocità cartesiana | `CartesianVelocityTracker` | lo stesso controllo sul twist; integra il twist come il robot |

Se Ruckig non trova un profilo che arrivi all'accelerazione obiettivo (succede quando è esattamente al limite), il tracker riprova puntando solo alla velocità obiettivo, che è sempre raggiungibile. Solo se fallisce anche questo il tracker conta un errore e tiene il comando.

## Il loop in modalità external

- Il primo comando continua quello attuale del robot: `q_d` per i giunti, `O_T_EE_c` per le pose, come negli esempi di libfranka.
- Ogni campione viene riprodotto `playout_delay_ms` dopo il suo stamp. Il tempo di riproduzione è il clock del robot (esattamente 1 ms per ciclo), allineato al clock del PC: così il jitter della rete non arriva al robot.
- Tra un campione e l'altro il riferimento è un fit quadratico locale (posizioni, pose, velocità) o un'interpolazione lineare (coppie). Per gli stream radi la finestra del fit si allarga da sola.
- Un campione accettabile per il robot viene inviato così com'è. Altrimenti il tracker lo adatta, e il ciclo viene contato in `limited_cycles`.
- I buchi brevi dello stream (meno di `stall_timeout_ms`) vengono coperti: posizioni, pose e velocità per estrapolazione ad accelerazione costante, coppie tenendo l'ultimo campione. Oltre, il robot si ferma dolcemente, con una frenata a jerk limitato (per le coppie con uno smorzamento `-D·dq`). Poi tiene la posizione; lo stream riparte con campioni nuovi.
- Uno stream di posizioni o di pose che parte a più di `max_jump` dal comando attuale viene rifiutato.
- Se l'end effector scende sotto `critical_height`, il robot si ferma dolcemente e il loop termina.

## Note sui bridge cartesiani

- Pose e twist sono nel frame base, come `O_T_EE` e `O_dP_EE` di libfranka: il `frame_id` dei messaggi non viene guardato. La posa è quella dell'EE impostato con `flange_T_ee_*`.
- Il segno del quaternione non conta: il bridge lo rende continuo da un campione all'altro.
- L'EE comandato non scende mai sotto `critical_height` + 1 cm. Uno stream che va più in basso viene saturato a quella quota (con `clamp` false ferma il robot), come i giunti ai finecorsa. Il controllo sull'EE misurato resta come ultima difesa.
- Gomito (`use_elbow_control`): se è false, il gomito lo sceglie il robot e `desired_elbow` viene ignorato. Se è true, il bridge comanda anche la posizione del giunto 3: durante gli stream la tiene ferma (i messaggi non hanno un campo per il gomito), i goal la muovono con `desired_elbow`. In questo caso indica sempre `desired_elbow`, `.nan` per tenere il gomito: il default del messaggio è 0, che è una posizione valida.
- I limiti cartesiani di default sono bassi (`safe_acceleration_factor` e `safe_jerk_factor` 0.3): con accelerazioni alte dell'EE sono i giunti a superare i loro limiti, e il bridge non lo vede. In ogni caso, vicino a una singolarità anche un moto lento dell'EE può far scattare un reflex sui giunti.

## Note sul bridge di coppia

- Le coppie del topic sono **senza gravità**, come in libfranka: la gravità la compensa il robot. Un controllore che include la gravità (per esempio uno scritto per la simulazione) la deve togliere, altrimenti viene applicata due volte.
- Quando non segue uno stream o un moto, il bridge tiene la posizione con un'impedenza di giunto: `joint_stiffness`, `joint_damping` e la coppia di Coriolis del modello. I guadagni di default sono quelli dell'esempio `joint_impedance_control` di libfranka.
- In modalità external il bridge non aggiunge niente: la coppia del topic va al robot, entro i limiti di coppia e di derivata.
- Un controllore esterno vede lo stato con qualche millisecondo di ritardo: con guadagni alti, soprattutto sugli ultimi giunti, il loop diventa instabile. Per una legge di controllo rigida usa la modalità internal.
- Il preset `gravity_compensation` lascia il robot libero di essere mosso a mano, fino a `/panda/stop_motion`. Le soglie di collisione di default (20 Nm) possono far scattare un reflex se lo spingi con forza.

## La Franka Hand

`GripperServer` (in `gripper_server.hpp/.cpp`) appartiene a `BridgeBase`: viene creato al Configure, se `use_gripper` è true e la pinza risponde, e distrutto al Cleanup. Parla con `franka::Gripper`, che ha una connessione sua (porta TCP 1338 per i comandi, UDP per lo stato): niente di quello che fa tocca il loop di controllo.

- In libfranka i comandi della pinza bloccano finché le dita si fermano. Girano in un thread worker, uno alla volta; il thread dell'executor del gruppo della pinza accetta solo i goal.
- `franka::Gripper::stop()` chiamato da un altro thread fa lanciare "Command aborted" al comando in corso. Lo usano il goal nuovo e il cancel: il goal nuovo aspetta in coda, quello vecchio finisce abortito (preempted) o cancellato.
- CLOSE senza larghezza è `grasp(0, speed, force, 0.005, 1.0)`: qualsiasi apertura finale vale come presa, quindi riesce sia su un oggetto sia a vuoto; il messaggio distingue i due casi (un'apertura sotto 2 mm vuol dire niente). A vuoto, poi, `stop()` toglie la forza: le dita restano chiuse senza premere l'una sull'altra.
- Su una pinza vera, le dita appoggiate al finecorsa esterno a volte non partivano: la presa, a forza limitata, si bloccava subito e la pinza dichiarava una presa alla stessa apertura. Per questo CLOSE, a meno di 1 mm dalla larghezza massima, prima sposta le dita di 2 mm con un movimento in posizione, che invece le fa partire.
- Un comando che doveva muovere le dita di più di 1 mm e le muove di meno fallisce, qualunque cosa dica la pinza. Un comando fallito va nel log come avviso; uno cancellato o sostituito no.
- Il risultato legge l'apertura da un aggiornamento di stato arrivato dopo la fine del comando, quindi è quella finale.
- Gli handle dei goal cambiano stato solo sotto un mutex, preso anche dal thread dello stato quando pubblica il feedback: il feedback non va mai a un goal appena finito.
- Al Cleanup un comando in corso viene fermato; una presa già fatta non viene rilasciata.
- `FakeGripperBackend` simula la pinza per `fake_robot`: dita che si muovono alla velocità comandata, e un oggetto facoltativo tra di esse (`fake_gripper_object_width`); nei test anche dita che si incastrano (`Jam`).

## Segnali e chiusura

Il bridge non si interrompe mai di colpo per cause esterne:

| Evento | Cosa fa il bridge |
|---|---|
| Ctrl+C (SIGINT), SIGTERM | fermata dolce, poi esce |
| chiusura del terminale (SIGHUP) | come Ctrl+C |
| la GUI che l'ha lanciato muore | continua a comandare il robot: il suo log va nel vuoto (SIGPIPE ignorato). La GUI successiva lo segnala allo Start |

Il bridge è un processo separato dalla GUI, nello stesso gruppo di processi: il Ctrl+C nel terminale del launch arriva a tutti e due. Il real-time non dipende da questo: il loop di libfranka gira in un thread SCHED_FIFO 99, che ha la precedenza su tutto il resto. Per isolarlo di più servono `rt_cpu` (una CPU dedicata), `lock_memory` e gli interrupt della NIC su quella CPU.

## Dove sono i bridge nel codice

Non c'è un file sorgente per ogni bridge: i bridge sono due famiglie di codice comune, istanziate per ogni tipo di comando.

| Parte | File | Comune o specifica |
|---|---|---|
| `main()` dell'eseguibile | `src/panda_{joint,cartesian}_*_bridge.cpp` | uno per bridge, solo `main()` |
| topic, azione, preset, status | `bridge/joint_bridge.hpp/.cpp`, `bridge/cartesian_bridge.hpp/.cpp` | una per famiglia; ciò che cambia tra i bridge della famiglia è nei `...InterfaceTraits` all'inizio dell'header |
| parti ROS comuni alle due famiglie | `bridge/bridge_common.hpp` | goal, eventi, statistiche dello stream, status |
| macchina a stati | `bridge/pipeline.hpp/.cpp` | comune a tutti |
| come diventa un comando accettabile | `bridge/joint_tracker.hpp`, `joint_velocity_tracker.hpp`, `joint_torque_tracker.hpp`, `cartesian_tracker.hpp` | uno per bridge (i due cartesiani nello stesso file) |
| connessione, thread, stato pubblicato | `bridge/bridge_base.hpp/.cpp` | comune a tutti |
| Franka Hand | `bridge/gripper_server.hpp/.cpp`, `bridge/gripper_backend.hpp/.cpp` | comune a tutti |

## File

### `uclv_panda_interfaces`

| File | Contenuto |
|---|---|
| `msg/JointsCommandedPositions.msg` | comando di posizioni di giunto |
| `msg/PandaRobotState.msg` | stato completo del robot |
| `msg/BridgeStatus.msg` | stato e diagnostica del bridge |
| `srv/RunPreset.srv` | avvio di un preset (nome, durata, ampiezza) |
| `action/JointTraj.action` | goal di giunto: posizioni e durata |
| `action/CartTraj.action` | goal cartesiano: posa, durata, gomito (NaN lo tiene) |
| `action/Gripper.action` | comando della Franka Hand: apri, chiudi (presa), muovi, homing |

### `panda_utils`: codice dei bridge

Gli header sono in `include/panda_utils/bridge/`, i sorgenti in `src/bridge/`.

| File | Contenuto |
|---|---|
| `types.hpp` | costanti (7 giunti, ciclo di 1 ms), `JointArray`, `JointFeedback` (stato del robot usato dai tracker di giunto), lettura dei clock |
| `franka_limits.hpp` | limiti del FER: di giunto, cartesiani e di coppia, dai valori di libfranka (`rate_limiting.h`) e da `robot_limits.hpp` |
| `cartesian_types.hpp` | pose, quaternioni, conversioni da e verso `O_T_EE`, `CartesianState` e `CartesianFeedback` |
| `robot_backend.hpp/.cpp` | interfaccia verso `franka::Robot` per i cinque tipi di comando; `FakeRobotBackend` è il robot simulato dei test |
| `gripper_backend.hpp/.cpp` | interfaccia verso `franka::Gripper`; `FakeGripperBackend` è la pinza simulata dei test |
| `gripper_server.hpp/.cpp` | la Franka Hand su `/panda/gripper` e `/panda/gripper_state`, per ogni bridge |
| `rt_exchange.hpp` | code lock-free e box per passare dati tra il loop e ROS |
| `state_conversions.hpp/.cpp` | da `franka::RobotState` ai messaggi; tabelle dello stato iniziale stampate al Configure |
| `bridge_base.hpp/.cpp` | nodo lifecycle comune a tutti i bridge: connessione, thread, publisher, recovery, Franka Hand |
| `bridge_main.hpp` | `main()` comune degli eseguibili: nodo ed executor |
| `bridge_common.hpp` | parti ROS comuni ai bridge: snapshot del loop, eventi e log, statistiche dello stream, status, goal delle azioni |
| `jitter_buffer.hpp` | campioni ordinati per stamp: fit quadratico locale (posizioni, pose, velocità) o interpolazione lineare (coppie) |
| `playback_clock.hpp` | tempo di riproduzione: clock del robot allineato a quello del PC |
| `pipeline_types.hpp` | tipi della macchina a stati: configurazione, contatori, stato dei moti, richieste |
| `pipeline.hpp/.cpp` | macchina a stati comune a tutti i bridge, senza ROS né libfranka; un'istanza per tracker |
| `joint_tracker.hpp` | tracker delle posizioni di giunto |
| `joint_velocity_tracker.hpp` | tracker delle velocità di giunto |
| `joint_torque_tracker.hpp` | tracker delle coppie: limiti, derivata, impedenza, smorzamento, hold |
| `cartesian_tracker.hpp` | tracker delle pose e dei twist, con quota minima e gomito |
| `joint_motions.hpp` | moti internal di giunto: quintica, "cosine bump", libero (gravity compensation) |
| `cartesian_motions.hpp` | moti internal cartesiani: quintica (retta e slerp), andata e ritorno su un asse, cerchio |
| `joint_bridge.hpp/.cpp` | i bridge di giunto: topic, azione, preset, status; un'istanza per posizioni, velocità e coppie |
| `cartesian_bridge.hpp/.cpp` | i bridge cartesiani: lo stesso, un'istanza per posa e velocità |

### `panda_utils`: altri file

| File | Contenuto |
|---|---|
| `src/panda_*_bridge.cpp` | i cinque eseguibili, solo `main()` |
| `src/simple_joint_traj.cpp` | client di riferimento delle posizioni di giunto (eseguibile `simple_joints_traj`) |
| `src/simple_velocity_joint_traj.cpp` | client di riferimento delle velocità di giunto (eseguibile `simple_velocity_traj`) |
| `src/simple_torque_cmd.cpp` | client di riferimento delle coppie: coppie nulle a 1 kHz |
| `src/simple_cartesian_traj.cpp` | client di riferimento dei bridge cartesiani: quintica fino a una posa (o di uno spostamento), come pose o come twist |
| `examples/joint_position_client.py` | esempio minimo di client Python, a 100 Hz |
| `include/panda_utils/robot_limits.hpp` | limiti del Panda, senza dipendenze ROS |
| `include/panda_utils/constants.hpp`, `utils_func.hpp` | nomi, QoS e funzioni geometriche comuni |
| `gui/panda_gui_py.py` | la GUI (Tkinter + rclpy) |
| `launch/gui.launch.py` | lancia la GUI |
| `src/panda_gui.cpp` | vecchia GUI in Qt, non più usata |

### Test (`panda_utils/test/`)

| File | Cosa verifica |
|---|---|
| `kinematics_checks.hpp` | helper: controlla i comandi di giunto e cartesiani come il robot (differenze finite, norme) |
| `test_jitter_buffer.cpp` | ordinamento, duplicati, fit, estrapolazione |
| `test_playback_clock.cpp` | jitter, deriva e salti del clock |
| `test_joint_tracker.cpp` | posizioni: riferimenti fattibili riprodotti esattamente, gradini e frenate nei limiti, finecorsa |
| `test_joint_velocity_tracker.cpp` | lo stesso per le velocità, più l'integrazione della posizione |
| `test_cartesian_tracker.cpp` | quaternioni e conversioni; pose e twist riprodotti esattamente, gradini e frenate nei limiti, quota minima, gomito |
| `test_joint_position_pipeline.cpp` | stream con jitter, perdite e duplicati, stream a gradini, stalli, timeout, moti internal, cancel |
| `test_joint_velocity_pipeline.cpp` | stream di velocità (anche uno che finisce esattamente a fine quintica, e uno a gradini), stalli, moti internal che arrivano alla posizione |
| `test_joint_torque_pipeline.cpp` | con un robot simulato: hold, stream con limiti e derivata, smorzamento, impedenza, gravity compensation |
| `test_cartesian_pipeline.cpp` | stream di pose (anche con il segno del quaternione invertito, e a gradini) e di twist, stalli, salti, quota minima, goal, cerchio |
| `test_bridges.cpp` | i cinque bridge completi sul robot simulato: cicli activate/deactivate, stream, goal e preset; la Franka Hand in ogni bridge e modalità: apertura, chiusura, presa, preemption, cancel, goal non validi |
| `test_gripper_server.cpp` | il server della Franka Hand sulla pinza simulata, con dita che si incastrano: staccate dal finecorsa, comandi che non le muovono falliti, forza tolta dopo una chiusura a vuoto |

I test girano su un dominio ROS isolato (215-232, solo localhost), quindi non raggiungono mai un bridge collegato al robot.

I linter di stile (uncrustify, flake8, pep257) sono esclusi in `CMakeLists.txt`: il codice non segue lo stile di formattazione di ROS, e fallirebbero su ogni file nascondendo i problemi veri. Restano attivi `lint_cmake`, `xmllint` e `cppcheck`.
