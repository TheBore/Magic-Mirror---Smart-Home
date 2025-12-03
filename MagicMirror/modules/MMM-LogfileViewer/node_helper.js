/* MagicMirror²
 * Node Helper: MMM-LogfileViewer
 *
 * By [Your Name]
 * MIT Licensed.
 */

const NodeHelper = require("node_helper");
const fs = require("fs");
const path = require("path");
const chokidar = require("chokidar");

module.exports = NodeHelper.create({
    start: function() {
        console.log("Starting node helper for: " + this.name);
        this.watchers = new Map(); // Track watchers per socket
    },

    stop: function() {
        console.log("Stopping node helper for: " + this.name);
        // Clean up all watchers
        this.watchers.forEach(watcher => {
            if (watcher) {
                watcher.close();
            }
        });
        this.watchers.clear();
    },

    socketNotificationReceived: function(notification, payload) {
        if (notification === "START_WATCHING") {
            this.startWatching(payload);
        }
    },

    startWatching: function(config) {
        // Create unique identifier based on file path and module instance
        const socketId = config.identifier || config.filePath;
        
        try {
            // Stop any existing watcher for this socket
            if (this.watchers.has(socketId)) {
                this.watchers.get(socketId).close();
            }

            // Validate file path
            if (!config.filePath) {
                this.sendSocketNotification("ERROR", {
                    message: "No file path specified",
                    details: "File path is required in configuration",
                    identifier: socketId
                });
                return;
            }

            // Resolve absolute path
            const filePath = path.resolve(config.filePath);
            
            // Check if file exists
            if (!fs.existsSync(filePath)) {
                this.sendSocketNotification("ERROR", {
                    message: "File not found",
                    details: `File does not exist: ${filePath}`,
                    identifier: socketId
                });
                return;
            }

            // Check if it's actually a file
            const stats = fs.statSync(filePath);
            if (!stats.isFile()) {
                this.sendSocketNotification("ERROR", {
                    message: "Path is not a file",
                    details: `Path is not a file: ${filePath}`,
                    identifier: socketId
                });
                return;
            }

            // Read initial content
            this.readLogFile(filePath, config.numberOfLines, socketId);

            // Set up optimal chokidar watcher
            const watcher = chokidar.watch(filePath, {
                persistent: true,              // Keep process running
                usePolling: false,             // Use native fs.watch (best performance)
                awaitWriteFinish: {            // Wait for chunked writes (critical for logs)
                    stabilityThreshold: 2000,  // Wait 2s after size stops changing
                    pollInterval: 100          // Check every 100ms
                },
                atomic: true,                  // Handle atomic writes (editor saves)
                ignoreInitial: false,          // Emit events for existing content
                ignorePermissionErrors: false  // Report permission issues
            });

            // Handle file changes
            watcher.on('change', (watchedPath) => {
                console.log(`File ${watchedPath} has been changed`);
                this.readLogFile(filePath, config.numberOfLines, socketId);
            });

            // Handle errors
            watcher.on('error', (error) => {
                console.error(`Watcher error: ${error}`);
                this.sendSocketNotification("ERROR", {
                    message: "File watching error",
                    details: error.message,
                    identifier: socketId
                });
            });

            // Handle file removal
            watcher.on('unlink', (watchedPath) => {
                console.log(`File ${watchedPath} has been removed`);
                this.sendSocketNotification("ERROR", {
                    message: "File was deleted",
                    details: `Log file was removed: ${watchedPath}`,
                    identifier: socketId
                });
            });

            // Store watcher
            this.watchers.set(socketId, watcher);

            console.log(`Started watching: ${filePath}`);

        } catch (error) {
            console.error("Error setting up file watcher:", error);
            this.sendSocketNotification("ERROR", {
                message: "Failed to start watching file",
                details: error.message,
                identifier: socketId
            });
        }
    },

    readLogFile: function(filePath, numberOfLines, socketId) {
        try {
            // Read file content
            const content = fs.readFileSync(filePath, 'utf8');
            
            // Split into lines and get last N lines
            const allLines = content.split('\n');
            
            // Remove empty last line if file ends with newline
            if (allLines.length > 0 && allLines[allLines.length - 1] === '') {
                allLines.pop();
            }
            
            // Get the last numberOfLines lines
            const lines = allLines.slice(-numberOfLines);
            
            // Send update to frontend
            this.sendSocketNotification("LOG_UPDATE", {
                lines: lines,
                totalLines: allLines.length,
                identifier: socketId
            });

        } catch (error) {
            console.error("Error reading log file:", error);
            
            // Handle specific error types
            let errorMessage = "Cannot read log file";
            if (error.code === 'ENOENT') {
                errorMessage = "File not found";
            } else if (error.code === 'EACCES') {
                errorMessage = "Permission denied";
            } else if (error.code === 'EISDIR') {
                errorMessage = "Path is a directory, not a file";
            }
            
            this.sendSocketNotification("ERROR", {
                message: errorMessage,
                details: error.message,
                identifier: socketId
            });
        }
    }
});