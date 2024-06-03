ssh 172.16.0.3 "pkill -f run_server.py"
ssh 172.16.0.3 "source ~/miniconda3/etc/profile.d/conda.sh; conda activate polymetis-local; cd /home/r2d2/droid_ws/src/DROID/scripts/server; python run_server.py"
ssh 172.16.0.3 "pkill -f run_server.py"
read -p "NUC server stopped. Press enter to continue"
