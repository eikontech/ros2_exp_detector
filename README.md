# ros2_exp_detector

Repository per il detector SPaCe

## comandi utili
- per scaricare il repository:
    ```git clone --recurse-submodule https://github.com/eikontech/ros2_exp_detector.git```
- per scaricare le dipendenze dei pacchetti, posizionarsi nella folder ros2_exp_detector e lanciare il comando:
    ```rosdep install --from-paths src --ignore-src -r -y```
- per compilare, digitare:
    ```colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --event-handlers console_direct+ --executor sequential --symlink-install```
- per eseguire:
    ```ros2 run <nome_pacchetto> <nome_nodo>```