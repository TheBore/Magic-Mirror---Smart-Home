Module.register("MMM-DHT20Display", {
  defaults: {
    wsUrl: "ws://localhost:8070" // Change this if your server is remote
  },

  start() {
    this.dataLoaded = false;
    this.dht20Data = {};
    this.sendSocketNotification("INIT_WEBSOCKET_DHT20", this.config.wsUrl);
  },

  getDom() {
    const wrapper = document.createElement("div");
    wrapper.className = "MMM-DHT20Display";

    if (!this.dataLoaded) {
      wrapper.innerHTML = "Waiting for DHT20 data...";
      return wrapper;
    }

    console.log(this.dht20Data)

    wrapper.innerHTML = `
      <h3>DHT20 Sensor Data</h3>
      <ul class="dht20-list">
        ${Object.entries(this.dht20Data).map(([key, value]) => {
  			let color = "inherit";
  			let emoji = "";

  			if (key === "heat_index_c") {
    			const result = this.getHeatIndexColorAndEmoji(parseFloat(value));
    			color = result.color;
    			emoji = result.emoji;
  			}

  			if (key === "humidex") {
    			const result = this.getHumidexColorAndEmoji(parseFloat(value));
    			color = result.color;
    			emoji = result.emoji;
  			}

  			return `
    			<li>
      				<strong class="key">${key}:</strong>
      				<span class="value" style="color: ${color};">${value} ${emoji}</span>
    			</li>`;
		}).join("")}
      </ul>
    `;
    return wrapper;
  },

  socketNotificationReceived(notification, payload) {
    if (notification === "DHT20_DATA") {
      this.dht20Data = payload;
      this.dataLoaded = true;
      this.updateDom();
    }
  },

  getStyles() {
    return ["MMM-DHT20Display.css"];
  },

  getHeatIndexColorAndEmoji(value) {
  	if (value < 27) return { color: "#44c767", emoji: "😊" };        // Normal
  	if (value < 32) return { color: "#f4d03f", emoji: "😐" };        // Caution
  	if (value < 41) return { color: "#f39c12", emoji: "🥵" };        // Extreme caution
  	if (value < 54) return { color: "#e74c3c", emoji: "🔥" };        // Danger
  	return { color: "#c0392b", emoji: "☠️" };                        // Extreme danger
  },

  getHumidexColorAndEmoji(value) {
  	if (value < 30) return { color: "#44c767", emoji: "😊" };        // Comfortable
  	if (value < 40) return { color: "#f4d03f", emoji: "😓" };        // Some discomfort
  	if (value <= 45) return { color: "#f39c12", emoji: "🥵" };       // Great discomfort
  	if (value <= 53) return { color: "#e74c3c", emoji: "🔥" };       // Dangerous
  	return { color: "#c0392b", emoji: "☠️" };                        // Extreme danger
  }

});

