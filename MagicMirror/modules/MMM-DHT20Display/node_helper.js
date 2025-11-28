const NodeHelper = require("node_helper");
const WebSocket = require("ws");

module.exports = NodeHelper.create({
  start() {
    this.ws = null;
    this.reconnectTimer = null;
    this.wsUrl = null;
  },

  socketNotificationReceived(notification, payload) {
    if (notification === "INIT_WEBSOCKET_DHT20") {
      const wsUrl = payload;
      
      // If already connected to the same URL, skip
      if (this.ws && this.wsUrl === wsUrl && this.ws.readyState === WebSocket.OPEN) {
        this.log("WebSocket already connected to:", wsUrl);
        return;
      }

      // Clean up existing connection before creating new one
      this.cleanupWebSocket();

      this.wsUrl = wsUrl;
      this.log(`Connecting to WebSocket at ${wsUrl}...`);

      this.ws = new WebSocket(wsUrl);

      this.ws.on("open", () => {
        this.log("WebSocket connection established.");
        // Clear any pending reconnect timer since we're connected
        if (this.reconnectTimer) {
          clearTimeout(this.reconnectTimer);
          this.reconnectTimer = null;
        }
      });

      this.ws.on("message", (message) => {
        this.log("Received message:", message);

        try {
          const data = JSON.parse(message);
          this.sendSocketNotification("DHT20_DATA", data);
        } catch (e) {
          this.log("Error parsing WebSocket message:", e);
        }
      });

      this.ws.on("error", (error) => {
        this.log("WebSocket error:", error);
      });

      this.ws.on("close", () => {
        this.log("WebSocket connection closed. Retrying in 5s...");
        // Clear old timer if it exists
        if (this.reconnectTimer) {
          clearTimeout(this.reconnectTimer);
        }
        // Schedule reconnect
        this.reconnectTimer = setTimeout(() => {
          this.reconnectTimer = null;
          if (this.wsUrl) {
            this.socketNotificationReceived("INIT_WEBSOCKET_DHT20", this.wsUrl);
          }
        }, 5000);
      });
    }
  },

  cleanupWebSocket() {
    // Clear reconnect timer
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }

    // Clean up existing WebSocket connection
    if (this.ws) {
      // Remove all event listeners to prevent memory leaks
      this.ws.removeAllListeners();
      
      // Close the connection if it's still open
      if (this.ws.readyState === WebSocket.OPEN || this.ws.readyState === WebSocket.CONNECTING) {
        this.ws.close();
      }
      
      this.ws = null;
    }
  },

  stop() {
    this.log("Stopping MMM-DHT20Display node helper...");
    this.cleanupWebSocket();
    this.wsUrl = null;
  },

  log(...args) {
    console.log("[MMM-DHT20Display]", ...args);
  }
});

