/* MagicMirror²
 * Module: MMM-LogfileViewer
 *
 * By [Your Name]
 * MIT Licensed.
 */

Module.register("MMM-LogfileViewer", {
    // Default module config
    defaults: {
        filePath: "",                    // Required: path to log file
        showTitle: true,                 // Optional: show title bar in the module_header
        titlePrefix: "Log Viewer for: ", // Optional: title prefix
        showFullPath: true,              // Optional: show full file path in title; if false shows only filename
        height: "300px",                 // Height of the log viewer
        width: "300px",                  // Width of the log viewer
        numberOfLines: 10,               // Lines to display
        fontSize: "medium",              // small|medium|large|"16px"|"1.2em"|any CSS font-size
        updateInterval: 1000,            // File polling interval (ms)
        logLevels: {                     // Color mapping with full CSS support
            ERROR: { color: "#ff6b6b", fontWeight: "bold" },
            WARN: { color: "#ffd93d", backgroundColor: "#333" }, 
            INFO: { color: "#6bcf7f" },
            DEBUG: { color: "#888", fontStyle: "italic" }
        },
        customStyles: {                  // Additional CSS rules for any element
            container: { padding: "10px", border: "1px solid #ccc", fontSize: "14px" },
            errorMessage: { color: "#ff0000", textAlign: "center" },
            logLine: { lineHeight: "1.4", fontFamily: "monospace" }
        }
    },

    // Initialize module
    start: function() {
        Log.info("Starting module: " + this.name);
        
        this.logLines = [];
        this.previousLines = [];
        this.error = null;
        this.isLoaded = false;
        this.logContainer = null;
        
        // Create unique identifier for this instance
        this.instanceId = this.identifier || this.config.filePath || (this.name + "_" + Date.now());

        // Validate required config
        if (!this.config.filePath) {
            this.error = "File path is required in configuration";
            Log.error(this.name, "No file path specified in config");
            return;
        }

        // Send configuration to node_helper with unique identifier
        const configWithId = Object.assign({}, this.config, { identifier: this.instanceId });
        this.sendSocketNotification("START_WATCHING", configWithId);
    },

    // Handle notifications from node_helper
    socketNotificationReceived: function(notification, payload) {
        // Only process notifications meant for this instance
        if (payload && payload.identifier && payload.identifier !== this.instanceId) {
            return;
        }
        
        if (notification === "LOG_UPDATE") {
            const newLines = payload.lines;
            this.error = null;
            
            if (!this.isLoaded) {
                // First load - show all lines
                this.logLines = newLines;
                this.previousLines = [...newLines];
                this.isLoaded = true;
                this.updateDom(300);
            } else {
                // Subsequent updates - add only new lines
                this.handleNewLines(newLines);
            }
        } else if (notification === "ERROR") {
            this.error = payload.message;
            this.logLines = [];
            this.previousLines = [];
            this.isLoaded = true;
            Log.error(this.name, "File error:", payload.details);
            this.updateDom(300);
        }
    },

    // Create the DOM structure
    getDom: function() {
        const wrapper = document.createElement("div");
        wrapper.className = "mmm-logviewer-container";

        // Apply custom container styles
        if (this.config.customStyles && this.config.customStyles.container) {
            this.applyStyles(wrapper, this.config.customStyles.container);
        }

        // Apply height and width if specified
        if (this.config.height) {
            wrapper.style.height = this.config.height;
        }
        if (this.config.width) {
            wrapper.style.width = this.config.width;
        }

        // Apply fontSize if not in customStyles
        if (this.config.fontSize && (!this.config.customStyles?.container?.fontSize)) {
            this.applyFontSize(wrapper, this.config.fontSize);
        }

        // Show loading message
        if (!this.isLoaded) {
            const loading = document.createElement("div");
            loading.className = "mmm-logviewer-loading";
            loading.innerHTML = "Loading log file...";
            wrapper.appendChild(loading);
            this.logContainer = null;
            return wrapper;
        }

        // Show error message
        if (this.error) {
            const errorDiv = document.createElement("div");
            errorDiv.className = "mmm-logviewer-error";
            errorDiv.innerHTML = this.error;
            
            // Apply custom error message styles
            if (this.config.customStyles && this.config.customStyles.errorMessage) {
                this.applyStyles(errorDiv, this.config.customStyles.errorMessage);
            }
            
            wrapper.appendChild(errorDiv);
            this.logContainer = null;
            return wrapper;
        }

        // Show log lines
        if (this.logLines.length === 0) {
            const emptyDiv = document.createElement("div");
            emptyDiv.className = "mmm-logviewer-empty";
            emptyDiv.innerHTML = "Log file is empty";
            wrapper.appendChild(emptyDiv);
            this.logContainer = null;
            return wrapper;
        }

        // Create log lines
        this.logLines.forEach(line => {
            const lineDiv = this.createLogLineElement(line);
            wrapper.appendChild(lineDiv);
        });

        // Store reference to container for incremental updates
        this.logContainer = wrapper;

        // Auto-scroll to bottom on initial load
        this.scrollToBottom();

        return wrapper;
    },

    // Return CSS files
    getStyles: function() {
        return ["MMM-LogfileViewer.css"];
    },

    // Return module header
    getHeader: function() {
        // If showTitle is false, return false to hide the header
        if (!this.config.showTitle) {
            return false;
        }

        if (!this.config.filePath) {
            return this.config.titlePrefix + "No file specified";
        }

        let displayPath;
        if (this.config.showFullPath) {
            displayPath = this.config.filePath;
        } else {
            // Extract filename from path (works for both Unix and Windows paths)
            displayPath = this.getFileNameFromPath(this.config.filePath);
        }

        return this.config.titlePrefix + displayPath;
    },

    // Extract filename from a file path
    getFileNameFromPath: function(filePath) {
        if (!filePath) return "";
        
        // Handle both Unix (/) and Windows (\) path separators
        const pathParts = filePath.split(/[/\\]/);
        return pathParts[pathParts.length - 1] || filePath;
    },

    // Handle new lines in a tail -f manner
    handleNewLines: function(newLines) {
        if (!this.logContainer) {
            // Container not ready, fall back to full update
            this.logLines = newLines;
            this.previousLines = [...newLines];
            this.updateDom(300);
            // Ensure scroll to bottom after DOM update
            setTimeout(() => this.scrollToBottom(), 350);
            return;
        }

        // Find new lines by comparing with previous state
        const previousLinesSet = new Set(this.previousLines);
        const actualNewLines = [];
        
        // Check if we have completely new content (file rotated/truncated)
        let hasCommonLines = false;
        for (let line of this.previousLines.slice(-5)) { // Check last 5 lines
            if (newLines.includes(line)) {
                hasCommonLines = true;
                break;
            }
        }

        if (!hasCommonLines && this.previousLines.length > 0 && newLines.length > 0) {
            // File seems to have been rotated or truncated, do full refresh
            this.logLines = newLines;
            this.previousLines = [...newLines];
            this.updateDom(300);
            // Ensure scroll to bottom after DOM update
            setTimeout(() => this.scrollToBottom(), 350);
            return;
        }

        // Find genuinely new lines
        for (let i = newLines.length - 1; i >= 0; i--) {
            const line = newLines[i];
            if (!previousLinesSet.has(line)) {
                actualNewLines.unshift(line);
            } else {
                break; // Stop when we find a line we've seen before
            }
        }

        if (actualNewLines.length > 0) {
            this.addNewLinesToDom(actualNewLines);
            this.logLines = newLines;
            this.previousLines = [...newLines];
            
            // Ensure we don't exceed the configured number of lines
            this.trimLinesToLimit();
        }
    },

    // Add new lines to DOM with animation
    addNewLinesToDom: function(newLines) {
        if (!this.logContainer) return;

        newLines.forEach(line => {
            const lineDiv = this.createLogLineElement(line);
            lineDiv.style.opacity = '0';
            lineDiv.style.transform = 'translateY(-10px)';
            
            this.logContainer.appendChild(lineDiv);
            
            // Animate the new line in
            setTimeout(() => {
                lineDiv.style.transition = 'opacity 0.3s ease, transform 0.3s ease';
                lineDiv.style.opacity = '1';
                lineDiv.style.transform = 'translateY(0)';
            }, 10);
        });

        // Auto-scroll to bottom after adding new lines
        this.scrollToBottom();
    },

    // Remove excess lines from the top to maintain numberOfLines limit
    trimLinesToLimit: function() {
        if (!this.logContainer) return;

        const maxLines = this.config.numberOfLines;
        const lineElements = this.logContainer.querySelectorAll('.mmm-logviewer-line');
        
        if (lineElements.length > maxLines) {
            const linesToRemove = lineElements.length - maxLines;
            for (let i = 0; i < linesToRemove; i++) {
                const lineToRemove = lineElements[i];
                lineToRemove.style.transition = 'opacity 0.2s ease, transform 0.2s ease';
                lineToRemove.style.opacity = '0';
                lineToRemove.style.transform = 'translateY(-10px)';
                
                setTimeout(() => {
                    if (lineToRemove.parentNode) {
                        lineToRemove.parentNode.removeChild(lineToRemove);
                    }
                }, 200);
            }
        }
    },

    // Create a log line element
    createLogLineElement: function(line) {
        const lineDiv = document.createElement("div");
        lineDiv.className = "mmm-logviewer-line";
        lineDiv.innerHTML = this.escapeHtml(line);

        // Apply custom log line styles
        if (this.config.customStyles && this.config.customStyles.logLine) {
            this.applyStyles(lineDiv, this.config.customStyles.logLine);
        }

        // Apply log level styling
        const logLevel = this.detectLogLevel(line);
        if (logLevel && this.config.logLevels[logLevel]) {
            this.applyStyles(lineDiv, this.config.logLevels[logLevel]);
            lineDiv.classList.add("mmm-logviewer-" + logLevel.toLowerCase());
        }

        return lineDiv;
    },

    // Scroll container to the very bottom
    scrollToBottom: function() {
        if (!this.logContainer) return;
        
        // Use setTimeout to ensure DOM has been updated
        setTimeout(() => {
            // Scroll to the absolute bottom
            this.logContainer.scrollTop = this.logContainer.scrollHeight;
        }, 50);
    },

    // Detect log level in a line using regex
    detectLogLevel: function(line) {
        const logLevelRegex = /(ERROR|WARN|WARNING|INFO|DEBUG)/i;
        const match = line.match(logLevelRegex);
        if (match) {
            const level = match[1].toUpperCase();
            // Normalize WARNING to WARN
            return level === "WARNING" ? "WARN" : level;
        }
        return null;
    },

    // Apply CSS styles to an element
    applyStyles: function(element, styles) {
        Object.keys(styles).forEach(property => {
            const camelCaseProperty = this.toCamelCase(property);
            element.style[camelCaseProperty] = styles[property];
        });
    },

    // Apply font size with support for predefined and custom values
    applyFontSize: function(element, fontSize) {
        const fontSizeMap = {
            "small": "0.9em",
            "medium": "1em",
            "large": "1.2em"
        };

        const actualFontSize = fontSizeMap[fontSize] || fontSize;
        element.style.fontSize = actualFontSize;
    },

    // Convert kebab-case to camelCase for CSS properties
    toCamelCase: function(str) {
        return str.replace(/-([a-z])/g, function(match, letter) {
            return letter.toUpperCase();
        });
    },

    // Escape HTML characters to prevent XSS
    escapeHtml: function(text) {
        const div = document.createElement('div');
        div.textContent = text;
        return div.innerHTML;
    }
});