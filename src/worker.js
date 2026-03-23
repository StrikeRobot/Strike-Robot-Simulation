let socket = null;
const WS_URL = "ws://localhost:8765";

// Handle messages from the main thread
self.onmessage = function (event) {
    const { type, data } = event.data;

    switch (type) {
        case 'init':
            connectWebSocket();
            break;

        case 'sendState':
            if (socket && socket.readyState === WebSocket.OPEN) {
                // Determine if we should log (debug)
                // data.time is sent along with other info
                if (Math.floor(data.time * 100) % 100 === 0) {
                    // console.log("Worker: Sending state at time", data.time.toFixed(2));
                }

                // Construct the payload
                const payload = {
                    time: data.time,
                    qpos: data.qpos,
                    qvel: data.qvel,
                    ctrl: data.ctrl,
                    qfrc_applied: data.qfrc_applied
                };

                socket.send(JSON.stringify(payload));
            }
            break;

        case 'close':
            if (socket) {
                socket.close();
            }
            break;
    }
};

function connectWebSocket() {
    console.log("Worker: Connecting to WebSocket at " + WS_URL);
    socket = new WebSocket(WS_URL);

    socket.onopen = () => {
        console.log("Worker: WebSocket connected");
        self.postMessage({ type: 'status', status: 'connected' });
    };

    socket.onmessage = (event) => {
        try {
            const msg = JSON.parse(event.data);
            // Post the parsed data back to the main thread
            self.postMessage({ type: 'update', data: msg });
        } catch (e) {
            console.error("Worker: Error parsing message", e);
        }
    };

    socket.onerror = (error) => {
        console.error("Worker: WebSocket Error", error);
        self.postMessage({ type: 'status', status: 'error' });
    };

    socket.onclose = () => {
        console.log("Worker: WebSocket closed");
        self.postMessage({ type: 'status', status: 'disconnected' });
        // Optional: Reconnection logic could go here
    };
}
