Module.register("MMM-ChatMock", {
  // Default module config.
  defaults: {
    updateInterval: 10 * 1000, // every 10 seconds
    maxMessages: 10,
  },

  start() {
    Log.info("Starting module: " + this.name);
    this.messages = [
      { from: "user", text: "Greetings. I have the following sensor data: DHT20: \{'timestamp': '2025-06-30 21:39:49', 'temperature_c': 28.72, 'humidity': 45.7, 'heat_index__c': 28.84, 'humidexi: 33.21\} SGP30: \{'timestamp': '2025-06-30 21:52:44', 'eco2_ppm': 400, 'tvoc_ppb': 15} SPS30: \{'mass_density_pm1.0': 7.35, 'mass_density_pm2.5': 7.77, 'mass_density_pm4.0': 7.77, 'mass_density_pm10': 7.77, 'particle_count_pm0.5': 49.77, 'particle_count_pm1.0': 58.47, 'particle_count_pm2.5': 58.64, 'particle_count_pm4.0': 58.65, 'particle_count_pm10': 58.66, 'particle_size': 0.43, 'mass_density_unit': 'ug/m3', 'particle_count_unit': '#/cm3', 'particle_size_unit': 'um\}'", time: '30.06.2025 08:01'},
      { from: "mirror", text: "Temp ~28.7°C & Humidity ~46%: Warm and moderately humid — stay hydrated and cool to avoid heat stress. Heat index ~28.8°C & Humidex ~33: Feels hotter than actual temp; risk of discomfort or mild heat exhaustion if active. eCO2 400 ppm & TVOC 15 ppb: Excellent indoor air quality — no concern. PM2.5 around 7.7 µg/m³: Low particulate pollution, safe for healthy and sensitive groups. Bottom line: Your environment is generally safe and comfortable but keep an eye on heat/humidity during activity to avoid overheating.", time: "30.06.2025 08:02" },
      { from: "user", text: "Greetings. I have the following sensor data: DHT20: \{'timestamp': '2025-06-30 21:39:49', 'temperature_c': 28.72, 'humidity': 45.7, 'heat_index__c': 28.84, 'humidexi: 33.21\} SGP30: \{'timestamp': '2025-06-30 21:52:44', 'eco2_ppm': 800, 'tvoc_ppb': 250} SPS30: \{'mass_density_pm1.0': 7.35, 'mass_density_pm2.5': 7.77, 'mass_density_pm4.0': 7.77, 'mass_density_pm10': 7.77, 'particle_count_pm0.5': 49.77, 'particle_count_pm1.0': 58.47, 'particle_count_pm2.5': 58.64, 'particle_count_pm4.0': 58.65, 'particle_count_pm10': 58.66, 'particle_size': 0.43, 'mass_density_unit': 'ug/m3', 'particle_count_unit': '#/cm3', 'particle_size_unit': 'um\}'", time: '30.06.2025 08:01'},
      { from: "mirror", text: "eCO2 at 800 ppm and TVOC at 250 ppb indicate moderate indoor air quality—ventilate if possible. Heat index and humidex values show mild heat stress risk; stay hydrated. PM values (~7.7 ug/m³) are low, so particulate pollution is fine. Regularly monitor and ventilate if VOC or CO2 rise. Bottom line: Data mostly looks safe but watch elevated VOC and CO2, plus heat/humidity discomfort.", time: "30.06.2025 08:04" },
    ];

    this.updateTimer = null;
  },

  getStyles() {
    return ["MMM-ChatMock.css"];
  },

  getDom() {
    const wrapper = document.createElement("div");
    wrapper.className = "chat-container";

	const header = document.createElement("h3");
    header.textContent = "OpenAI Chat";
    header.style.textAlign = "left";

    wrapper.appendChild(header);

    this.messages.forEach((msg, index) => {
      const msgDiv = document.createElement("div");
      msgDiv.className = "message " + (msg.from === "user" ? "user" : "mirror");
      msgDiv.textContent = msg.text;

      const timeDiv = document.createElement("div");
      timeDiv.className = index%2 !== 0 ? "timestamp-left" : "timestamp-right";
      timeDiv.textContent = msg.time;

      wrapper.appendChild(msgDiv);
      wrapper.appendChild(timeDiv);
    });

    return wrapper;
  },
});

