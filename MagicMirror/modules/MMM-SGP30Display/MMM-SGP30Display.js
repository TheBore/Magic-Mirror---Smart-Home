Module.register("MMM-SGP30Display", {
  defaults: {
    wsUrl: "ws://localhost:8071" // Change this if your server is remote
  },

  start() {
    this.dataLoaded = false;
    this.sgp30Data = {};
    this.sendSocketNotification("INIT_WEBSOCKET_SGP30", this.config.wsUrl);
  },

  getDom() {
    const wrapper = document.createElement("div");
    wrapper.className = "MMM-SGP30Display";

    if (!this.dataLoaded) {
      wrapper.innerHTML = "Waiting for SGP30 data...";
      return wrapper;
    }

    console.log(this.sgp30Data)

    wrapper.innerHTML = `
      <h3>SGP30 Sensor Data</h3>
      <ul class="sgp30-list">
        ${Object.entries(this.sgp30Data).map(([key, value]) => {
  			let color = "inherit";
  			let emoji = "";

  			if (key === "eco2_ppm") {
    			const result = this.getEco2ColorAndEmoji(parseFloat(value));
    			color = result.color;
    			emoji = result.emoji;
  			}

  			if (key === "tvoc_ppb") {
    			const result = this.getTvocColorAndEmoji(parseFloat(value));
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
    if (notification === "SGP30_DATA") {
      this.sgp30Data = payload;
      this.dataLoaded = true;
      this.updateDom();
    }
  },

  getStyles() {
    return ["MMM-SGP30Display.css"];
  },

  	getEco2ColorAndEmoji(value) {
  		if (value < 600) return { color: "#44c767", emoji: "😊" };
  		if (value < 1000) return { color: "#f4d03f", emoji: "🙂" };
  		if (value < 1500) return { color: "#f39c12", emoji: "😐" };
  		if (value < 2000) return { color: "#e67e22", emoji: "😓" };
  		if (value <= 5000) return { color: "#e74c3c", emoji: "😷" };
  		return { color: "#c0392b", emoji: "☠️" };
	},

	getTvocColorAndEmoji(value) {
  		if (value < 65) return { color: "#44c767", emoji: "😊" };
  		if (value < 220) return { color: "#f4d03f", emoji: "🙂" };
  		if (value < 660) return { color: "#f39c12", emoji: "😐" };
  		if (value < 2200) return { color: "#e67e22", emoji: "🤢" };
  		if (value < 3000) return { color: "#e74c3c", emoji: "😷" };
  		return { color: "#c0392b", emoji: "☠️" };
	}

});

