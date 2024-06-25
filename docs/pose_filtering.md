the program is located in src/task/scripts/visualize.py

start the docker first using docker compose, from the readme (docker.md) :
to launch the program, open the container's terminal, and execute the command below :
```sh
    rosrun task visualize.py
```

after that, open RVIZ in the host's terminal by:
```sh
    rviz
```

after that add Path, and set the topic into "/simplified_path" and "/raw_path"
the visualization will show.