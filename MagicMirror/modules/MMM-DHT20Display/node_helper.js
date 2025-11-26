const NodeHelper = require("node_helper");
const WebSocket = require("ws");

module.exports = NodeHelper.create({
  start() {
    this.ws = null;
  },

  socketNotificationReceived(notification, payload) {
    if (notification === "INIT_WEBSOCKET_DHT20") {
      const wsUrl = payload;
      this.log(`Connecting to WebSocket at ${wsUrl}...`);

      this.ws = new WebSocket(wsUrl);

      this.ws.on("open", () => {
        this.log("WebSocket connection established.");
      });

      this.ws.on("message", (message) => {

        this.log("Received message:", message);

        try {
          const data = JSON.parse(message);
          this.sendNotification("DHT20_DATA", data);
        } catch (e) {
          this.log("Error parsing WebSocket message:", e);
        }
      });

      this.ws.on("error", (error) => {
        this.log("WebSocket error:", error);
      });

      this.ws.on("close", () => {
        this.log("WebSocket connection closed. Retrying in 5s...");
        setTimeout(() => this.socketNotificationReceived("INIT_WEBSOCKET_DHT20", wsUrl), 5000);
      });
    }
  },

  log(...args) {
    console.log("[MMM-DHT20Display]", ...args);
  }
});

