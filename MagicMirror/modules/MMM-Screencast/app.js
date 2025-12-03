const electron = require('electron');
const Positioner = require('electron-positioner');
const { IpcServer } = require('./ipc.js');
const { POSITIONS } = require('./constants.js');

// more useragents here: https://developers.whatismybrowser.com/useragents/explore/operating_platform/smart-tv/
const userAgent = 'Mozilla/5.0 (SMART-TV; Linux; Tizen 2.4.0) AppleWebkit/538.1 (KHTML, like Gecko) SamsungBrowser/1.1 TV Safari/538.1';
const ipcInstance = new IpcServer();
const app = electron.app;

// Disable hardware acceleration for headless/Xvfb compatibility
// This is important when running on systems without a physical display
if (process.env.ELECTRON_ENABLE_GPU !== "1") {
  app.disableHardwareAcceleration();
}

// Add command line switches for better Xvfb/headless compatibility
app.commandLine.appendSwitch('autoplay-policy', 'no-user-gesture-required');
app.commandLine.appendSwitch('disable-gpu');
app.commandLine.appendSwitch('disable-software-rasterizer');
app.commandLine.appendSwitch('disable-dev-shm-usage'); // Helps with limited /dev/shm on some systems
// Suppress DBus errors on headless systems
app.commandLine.appendSwitch('disable-features', 'VizDisplayCompositor');
// Additional flags for Xvfb compatibility
app.commandLine.appendSwitch('no-sandbox');
app.commandLine.appendSwitch('disable-setuid-sandbox');

ipcInstance.on('QUIT', (data, socket) => {
  ipcInstance.emit(socket, 'QUIT_HEARD', {});
  app.quit();
  process.exit();
});

app.once('ready', () => {
  electron.session.defaultSession.setUserAgent(userAgent);

  ipcInstance.on('SEND_CONFIG', (data, socket) => {
    const { url, position, width, height, x, y } = data;

    console.log('[MMM-Screencast] Creating window with config:', { url, position, width, height, x, y });

    const usingXY = x && y;

    // electron
    const windowOptions = {
      maxHeight: height,
      maxWidth: width,
      resize: false,
      width: width,
      height: height,
      darkTheme: true,
      alwaysOnTop: true, // Fixed typo: was 'alwayOnTop'
      show: false,
      frame: false,
      zoomFactor: 1.0,
      focusable: false,
      webPreferences: {
        nodeIntegration: false,
        contextIsolation: true
      },
      ...(usingXY ? { x, y } : {})
    };

    let screenCastWindow;
    try {
      screenCastWindow = new electron.BrowserWindow(windowOptions);
      console.log('[MMM-Screencast] Window created successfully');
    } catch (error) {
      console.error('[MMM-Screencast] Error creating window:', error);
      ipcInstance.emit(socket, 'APP_READY', { error: error.message });
      return;
    }

    // Handle window errors
    screenCastWindow.webContents.on('did-fail-load', (event, errorCode, errorDescription) => {
      console.error('[MMM-Screencast] Failed to load URL:', errorCode, errorDescription);
    });

    screenCastWindow.on('closed', () => {
      console.log('[MMM-Screencast] Window closed');
    });

    if (!usingXY && POSITIONS[position]) {
      try {
        const positioner = new Positioner(screenCastWindow);
        positioner.move(POSITIONS[position]);
        console.log('[MMM-Screencast] Window positioned to:', POSITIONS[position]);
      } catch (error) {
        console.error('[MMM-Screencast] Error positioning window:', error);
        // Continue anyway - window will still work
      }
    }

    screenCastWindow.loadURL(url);
    console.log('[MMM-Screencast] Loading URL:', url);

     // Show window when page is ready
    screenCastWindow.once('ready-to-show', () => {
      console.log('[MMM-Screencast] Window ready to show');

      // this is messy for autoplay but youtube, due to chrome no longer supports
      // autoplay
      const autoPlayScript = `
        const videoEle = document.getElementsByTagName('video');
        if (!!videoEle && videoEle.length > 1) videoEle[0].play();
      `;

        //
        // THIS MIGHT NEED WORK
        //
        // maybe something like this:
        //
        // win.webContents.on('console-message', () => {
        //  // do the shit to cloes the window like above
        // })
        //
        // create a specific message or something to know that shit's done, or listen to whatever
        // https://electronjs.org/docs/api/web-contents#event-console-message
        //
        // ipc.server.on('screenCastWindow_config', (data, socket) => {
        //   const { extraScript, closeOnEnd } = data;
        //   const doScript = `${extraScript} ${closeOnEnd ? autoCloseScript : ''}`;
        //   screenCastWindow.webContents.executeJavaScript(doScript, true);

        //   ipc.server.emit(socket, 'quit');
        //   app.quit();
        //   process.exit();
        // });

        // ipc.server.broadcast('screenCastWindow_shown', { show: true });

      const autoCloseScript = `
        let videoEleStop;

       // consistently check the DOM for the video element
        const interval = setInterval(() => {
          videoEleStop = document.getElementsByTagName('video')[0];

         // if the video element exists add an event listener to it and stop the interval
          if (videoEleStop) {
            videoEleStop.addEventListener('ended', (event) => {
              console.log("mmm-screencast.exited");
            });
            clearInterval(interval);
          }
        }, 1000);
      `;

      screenCastWindow.webContents.on('console-message', (event, level, message, line, sourceId) => {
        if (message === "mmm-screencast.exited") {
           ipcInstance.server.emit(socket, 'quit');
           app.quit();
        }
      });

      try {
        screenCastWindow.show();
        screenCastWindow.focus(); // Ensure window gets focus
        console.log('[MMM-Screencast] Window shown');
        
        // Verify window is actually visible
        if (screenCastWindow.isVisible()) {
          console.log('[MMM-Screencast] Window is visible');
        } else {
          console.warn('[MMM-Screencast] Window show() called but isVisible() returns false');
        }
        
        // screenCastWindow.webContents.openDevTools();
        screenCastWindow.webContents.executeJavaScript(autoPlayScript, true);
        screenCastWindow.webContents.executeJavaScript(autoCloseScript, true);
        ipcInstance.emit(socket, 'APP_READY', {});
      } catch (error) {
        console.error('[MMM-Screencast] Error showing window:', error);
        ipcInstance.emit(socket, 'APP_READY', { error: error.message });
      }
    });

    // Also handle page load errors
    screenCastWindow.webContents.on('did-finish-load', () => {
      console.log('[MMM-Screencast] Page finished loading');
    });
  });
});
