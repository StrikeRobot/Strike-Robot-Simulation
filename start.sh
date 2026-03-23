
# # Start the WebSocket server in the background
# echo "Starting Socket Server on port 8765..."
# python3 socket_server.py &

# Start the HTTP server in the foreground (keeps container alive)
echo "Starting HTTP Server on port 8000..."
python3 run_server.py
