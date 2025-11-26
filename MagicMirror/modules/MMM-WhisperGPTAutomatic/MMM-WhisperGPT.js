/* global Module */

/* Magic Mirror
 * Module: MMM-WhisperGPTAutomatic
 *
 * By Sergiu Nagailic
 * MIT Licensed.
 */
Module.register("MMM-WhisperGPTAutomatic", {
	defaults: {
    state: 'idle',
    updateInterval: 60000, // Update every 60 seconds (1 minute)
    openAiKey: '',
    openAiSystemMsg: "You are a helpful AI assistant that analyzes environmental sensor data. Provide concise, friendly insights about temperature, humidity, and comfort levels based on the DHT20 sensor readings.",
    debug: false
	},

	requiresVersion: "2.1.0", // Required version of MagicMirror

	start: function() {
    Log.info("Starting module: " + this.name);
    this.state = 'idle';
    this.dht20Data = null;
    this.lastUpdateTime = null;

    const defaultConfig = {
      openAiKey: '',
      openAiSystemMsg: "You are a helpful AI assistant that analyzes environmental sensor data. Provide concise, friendly insights about temperature, humidity, and comfort levels based on the DHT20 sensor readings.",
      updateInterval: 60000, // Update every 60 seconds
      debug: false
    };

    // Merge default configuration with changed values.
    this.config = Object.assign({}, defaultConfig, this.config);

    // Check if required things are set.
    if (!this.config.openAiKey) {
      this.state = 'config_issue';
    }
    else {
      this.sendSocketNotification('CONFIG', this.config);
    }
	},

  getDom: function() {
    var wrapper = document.createElement("div");

    // State-based UI rendering
    switch(this.state) {
      case 'config_issue':
        wrapper.innerHTML = "Please supply configs (openAiKey)...";
        break;
      case 'idle':
        wrapper.innerHTML = "Waiting for DHT20 sensor data...";
        break;
      case 'processing':
        wrapper.innerHTML = "Processing sensor data with AI...";
        break;
      case 'data_received':
        if (this.dht20Data) {
          wrapper.innerHTML = '<div><span class="bright">Sensor Data: </span></div>';
          wrapper.innerHTML += '<div class="small">' + JSON.stringify(this.dht20Data, null, 2) + '</div>';
        }
        break;
      case 'reply_received':
        wrapper.innerHTML = '<div><span class="bright">AI Analysis: </span>' + this.replyText + '</div>';
        break;
      case 'error':
        wrapper.innerHTML = '<div><span class="bright">ERROR: </span>' + this.error + '</div>';
        // Reset state in 10 seconds.
        setTimeout(this.resetState.bind(this), 10 * 1000);
        break;
      default:
        wrapper.innerHTML = "Unknown state";
        break;
    }

    return wrapper;
  },

  getHeader: function() {
    return 'WhisperGPT Automatic';
  },


  getScripts: function() {
		return [];
	},

	getStyles: function () {
		return [
			"MMM-WhisperGPT.css",
		];
	},

	// Load translations files
	getTranslations: function() {
		//FIXME: This can be load a one file javascript definition
		return {
			en: "translations/en.json",
			es: "translations/es.json"
		};
	},

	processData: function(data) {
		var self = this;
		this.dataRequest = data;
		if (this.loaded === false) { self.updateDom(self.config.animationSpeed) ; }
		this.loaded = true;
	},


	notificationReceived: function(notification, payload) {
    if (notification === "DHT20_DATA") {
      Log.info('DHT20 data received: ', payload);
      this.dht20Data = payload;
      this.lastUpdateTime = Date.now();
      this.state = 'data_received';
      this.sendSocketNotification('PROCESS_DHT20_DATA', payload);
      this.updateDom();
    }
  },

	// socketNotificationReceived from helper
	socketNotificationReceived: function (notification, payload) {
    if (notification === 'PROCESSING') {
      Log.info('Processing DHT20 data with AI...');
      this.state = 'processing';
    }
    else if (notification === "ERROR") {
      this.state = 'error';
      this.error = payload;
    }
    else if (notification === 'REPLY_RECEIVED') {
      Log.info('AI Reply: ' + payload);
      this.state = 'reply_received';
      this.replyText = payload;
    }
    this.updateDom();
	},

  calculateDisplayTime: function (text, wordsPerMinute) {
    const words = text.split(' ').length;
    const minutes = words / wordsPerMinute;
    const seconds = minutes * 60;
    return seconds;
  },

  resetState: function() {
	  if (this.state === 'reply_received') {
      this.state = 'idle';
      this.replyText = '';
      this.dht20Data = null;
      this.updateDom();
    }
  }
});
