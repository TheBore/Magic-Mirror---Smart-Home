const NodeHelper = require('node_helper');
const Log = require("logger");

// ChainLang.
const { ConversationChain } = require("langchain/chains");
const { ChatOpenAI } = require("langchain/chat_models/openai");
const { BufferMemory } = require("langchain/memory");
const {  ChatPromptTemplate,
  HumanMessagePromptTemplate,
  SystemMessagePromptTemplate,
  MessagesPlaceholder } = require("langchain/prompts");

module.exports = NodeHelper.create({
  start: function() {
    console.log("Starting node_helper for: " + this.name);
    this.lastProcessedTime = null;
    this.processingInterval = null;
  },

  socketNotificationReceived: function(notification, payload) {
    if (notification === 'CONFIG') {
      this.config = payload;
      this.state = 'idle';
      this.chain = this.initGPT();
      
      Log.info("MMM-WhisperGPTAutomatic configured and ready to process DHT20 data");
    }
    else if (notification === 'PROCESS_DHT20_DATA') {
      // Process DHT20 data automatically
      this.processDHT20Data(payload);
    }
  },

  processDHT20Data: async function(dht20Data) {
    // Check if we should process (respect update interval)
    const now = Date.now();
    if (this.lastProcessedTime && (now - this.lastProcessedTime) < this.config.updateInterval) {
      if (this.config.debug) {
        Log.info('Skipping processing - too soon since last update');
      }
      return;
    }

    this.lastProcessedTime = now;
    this.state = 'processing';
    this.sendSocketNotification('PROCESSING');

    try {
      // Format the DHT20 data into a readable prompt
      const dataPrompt = this.formatDHT20Prompt(dht20Data);
      
      // Get AI reply
      const replyText = await this.getGPTReply(dataPrompt);
      
    } catch (e) {
      Log.error('Error processing DHT20 data:', e);
      this.sendSocketNotification('ERROR', 'Error processing sensor data with AI.');
    }
  },

  formatDHT20Prompt: function(dht20Data) {
    // Create a natural language prompt from the sensor data
    let prompt = "Here are the current environmental sensor readings from a DHT20 sensor:\n\n";
    
    if (dht20Data.temperature_c !== undefined) {
      prompt += `Temperature: ${dht20Data.temperature_c}°C\n`;
    }
    if (dht20Data.temperature_f !== undefined) {
      prompt += `Temperature: ${dht20Data.temperature_f}°F\n`;
    }
    if (dht20Data.humidity !== undefined) {
      prompt += `Humidity: ${dht20Data.humidity}%\n`;
    }
    if (dht20Data.heat_index_c !== undefined) {
      prompt += `Heat Index: ${dht20Data.heat_index_c}°C\n`;
    }
    if (dht20Data.humidex !== undefined) {
      prompt += `Humidex: ${dht20Data.humidex}\n`;
    }
    
    prompt += "\nPlease provide a brief, friendly analysis of these environmental conditions. Include any comfort level assessments and recommendations if appropriate.";
    
    return prompt;
  },


  initGPT: function () {
    const chat = new ChatOpenAI({
      openAIApiKey: this.config.openAiKey,
      temperature: 0.9,
      modelName: "gpt-3.5-turbo",
      streaming: false
    });

    const chatPrompt = ChatPromptTemplate.fromPromptMessages([
      SystemMessagePromptTemplate.fromTemplate(
        this.config.openAiSystemMsg
      ),
      new MessagesPlaceholder("history"),
      HumanMessagePromptTemplate.fromTemplate("{input}"),
    ]);

    return new ConversationChain({
      memory: new BufferMemory({ returnMessages: true, memoryKey: "history", aiPrefix: "AI" }),
      prompt: chatPrompt,
      llm: chat,
    });
  },

  getGPTReply: async function(requestText) {
    console.log('Sending request to OpenAPI: ' + requestText);
    try {
      const response = await this.chain.call({
        input: requestText,
      });
      console.log('OpenAI Response:');
      console.log(response);
      this.sendSocketNotification('REPLY_RECEIVED', response.response);

      return response.response;
    }
    catch (e) {
      this.sendSocketNotification('ERROR', 'Error from ChatGPT API.');
    }
  }
});
