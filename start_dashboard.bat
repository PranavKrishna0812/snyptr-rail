@echo off
echo Starting Target Rig Dashboard...
start http://localhost:8000/dashboard.html
python -m http.server 8000
