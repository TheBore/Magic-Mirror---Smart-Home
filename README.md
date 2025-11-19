### Steps to run everything

1. `tmux` - open tmux session
2. `cd ~/dht20` -> `source venv/bin/activate` -> `python3 DHT20_hi_humi_index_api.py` - run dht20
3. `ctrl + B + C` -> create new tmux tab
4. `cd ~/sgp30` -> `source venv/bin/activate` -> `python3 SGP30.py` - run sgp30
5. `ctrl + B + C` -> create new tmux tab
6. `cd ~/ros2_ws` -> `source install/setup.bash` - init ros2
7. `ros2 run sps30_publisher sps30_publisher` - run sps30 publisher
8. `ctrl + B + C` -> create new tmux tab
9. `cd ~/ros2_ws` -> `source install/setup.bash` - init ros2
10. `ros2 run sps30_listener sps30_listener` - run sps30 listener
11. `ctrl + B + C` -> create new tmux tab
12. `cd ~/MagicMirror` -> `npm run server` - start Magic Mirror as server, then access it on http://localhost:8080 on browser (should also setup SSH tunnel in Putty):

<img width="450" height="438" alt="image" src="https://github.com/user-attachments/assets/2acbe673-cc6a-4eaf-b93f-148904d85a4c" />

13. Use voice command `JARVIS`, then ask a question and wait for reply :)
