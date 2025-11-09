Module.register("MMM-SPS30Display", {
  defaults: {
    wsUrl: "ws://localhost:8069" // Change this if your server is remote
  },

  start() {
    this.dataLoaded = false;
    this.sps30Data = {};
    this.sendSocketNotification("INIT_WEBSOCKET_SPS30", this.config.wsUrl);
  },

  getDom() {
  	const wrapper = document.createElement("div");
  	wrapper.className = "MMM-SPS30Display";

  	if (!this.dataLoaded) {
    	wrapper.innerHTML = "Waiting for SPS30 data...";
    	return wrapper;
  	}

  	function getColorAndEmoji(key, value) {
    	if (typeof value !== "number") return { color: "", emoji: "" };

    	switch (key) {
      	case "mass_density_pm1.0":
      	case "mass_density_pm2.5":
        	if (value <= 12) return { color: "green", emoji: "😃" };
        	if (value <= 35.4) return { color: "yellow", emoji: "🙂" };
        	if (value <= 55.4) return { color: "orange", emoji: "😐" };
        	if (value <= 150.4) return { color: "red", emoji: "😷" };
        	return { color: "purple", emoji: "☠️" };

      	case "mass_density_pm4.0":
        	if (value <= 20) return { color: "green", emoji: "😃" };
        	if (value <= 50) return { color: "yellow", emoji: "🙂" };
        	if (value <= 100) return { color: "orange", emoji: "😐" };
        	if (value <= 150) return { color: "red", emoji: "😷" };
        	return { color: "purple", emoji: "☠️" };

      	case "mass_density_pm10":
        	if (value <= 54) return { color: "green", emoji: "😃" };
        	if (value <= 154) return { color: "yellow", emoji: "🙂" };
        	if (value <= 254) return { color: "orange", emoji: "😐" };
        	if (value <= 354) return { color: "red", emoji: "😷" };
        	return { color: "maroon", emoji: "☠️" };

      	default:
        	return { color: "", emoji: "" };
		}
  	}

  	wrapper.innerHTML = `
  		<h3>SPS30 Sensor Data</h3>
  		<ul class="sps30-list">
    	${Object.entries(this.sps30Data).map(([key, value]) => {
      		const numericValue = parseFloat(value);
      		const { color, emoji } = getColorAndEmoji(key, numericValue);
      		const style = color ? `color:${color}` : "";
      		return `<li><strong class="key">${key}:</strong>
                	<span class="value" style="${style}">${value} ${emoji}</span>
            	  </li>`;
    		}).join("")}
		  </ul>`;

  	return wrapper;
  },


  socketNotificationReceived(notification, payload) {
    if (notification === "SPS30_DATA") {
      this.sps30Data = payload;
      this.dataLoaded = true;
      this.updateDom();
    }
  },

  getStyles() {
    return ["MMM-SPS30Display.css"];
  }

});

