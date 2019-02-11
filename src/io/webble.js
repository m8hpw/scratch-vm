// const log = require('../util/log');

class WEBBLE {

    /**
     * A BT peripheral socket object.  It handles connecting, over web sockets, to
     * BT peripherals, and reading and writing data to them.
     * @param {Runtime} runtime - the Runtime for sending/receiving GUI update events.
     * @param {string} extensionId - the id of the extension using this socket.
     * @param {object} peripheralOptions - the list of options for peripheral discovery.
     * @param {object} connectCallback - a callback for connection.
     * @param {object} messageCallback - a callback for message sending.
     */
    constructor (runtime, extensionId, peripheralCHAR,peripheralSERV, connectCallback, parent) {
        this._availablePeripherals = {};
        this._connectCallback = connectCallback;
        this._connected = false;
        this._extensionId = extensionId;
        this._peripheralSERV = peripheralSERV;
        this._peripheralCHAR = peripheralCHAR;
        this._discoverTimeoutID = null;
        this._runtime = runtime;
        this._server = null;
        this.requestPeripheral();
		this._parent = parent;
		this._device = null;
        
    }
    
    
    /**
     * Sets LED mode and initial color and starts reading data from peripheral after BLE has connected.
     * @private
     */
    _onConnect () {
        //this.setLED(0x00);
        this.startNotifications(
            this._peripheralSERV,
            this._peripheralCHAR);
    }

    /**
     * Request connection to the peripheral.
     * If the web socket is not yet open, request when the socket promise resolves.
     */
    requestPeripheral () {
		if (typeof navigator.bluetooth != "undefined")
			return navigator.bluetooth.requestDevice({filters: [{services: [this._peripheralSERV]}],})
			.then(device => {device.addEventListener('gattserverdisconnected', this._sendDisconnectError);
							 device.parentBLE = this;
							 this._device = device;return device.gatt.connect();}) 
			.then(server => {this._server = server;this.connectPeripheral();})
        .catch(e => this._ConnectionError(e));
		else
		{
		//  TODO DIFFERENCIATE ERRORS FROM WEB BLE AND SCRATCH LINK
		 this._runtime.emit(this._runtime.constructor.PERIPHERAL_REQUEST_ERROR, {
            message: "WEB Bluetooth not enabled or not supported (see https://www.chromestatus.com/features/5264933985976320)",
            extensionId: this._extensionId
        });
        console.log("WEB Bluetooth not enabled or not supported (see https://www.chromestatus.com/features/5264933985976320)");
		}
      
    }

    /**
     * Try connecting to the input peripheral id, and then call the connect
     * callback if connection is successful.
     * @param {number} id - the id of the peripheral to connect to
     */
    connectPeripheral () {
			this._connected = true;
			this._runtime.emit(this._runtime.constructor.PERIPHERAL_CONNECTED);
            this._onConnect();
    }

    /**
     * Close the websocket.
     */
    disconnect () {
		if (this._device != null)
			this._device.gatt.disconnect()
    }

    /**
     * @return {bool} whether the peripheral is connected.
     */
    isConnected () {
		if (this._device !=null)
			return this._device.gatt.connected;
		else
			return false;
    }
    
   _characteristicDidChangeCallback (event) {
		value = event.target.value;
		boost = event.target.boost;
		boost.onMessage(value);
	}

    startNotifications (serviceId, characteristicId) {
        return this._server.getPrimaryService(serviceId)
				.then(service => service.getCharacteristic(characteristicId))
				.then(characteristic => { 
					characteristic.startNotifications().then(() => {
						characteristic.boost = this._parent;
						characteristic.addEventListener('characteristicvaluechanged',this._characteristicDidChangeCallback);
					    console.log('Notification started');

					});
				})
				.catch(e => this._sendRequestError(e));
    }

    /**
     * Read from the specified ble service.
     * @param {number} serviceId - the ble service to read.
     * @param {number} characteristicId - the ble characteristic to read.
     * @param {boolean} optStartNotifications - whether to start receiving characteristic change notifications.
     * @param {object} onCharacteristicChanged - callback for characteristic change notifications.
     * @return {Promise} - a promise from the remote read request.
     */
    read (serviceId, characteristicId, optStartNotifications = false, onCharacteristicChanged = null) {
        if (optStartNotifications) {
            params.startNotifications = true;
        }
   //     this._characteristicDidChangeCallback = onCharacteristicChanged;
   //     return this.sendRemoteRequest('read', params)
   //         .catch(e => {
   //             this._sendDisconnectError(e);
   //         });
    }

    /**
     * Write data to the specified ble service.
     * @param {number} serviceId - the ble service to write.
     * @param {number} characteristicId - the ble characteristic to write.
     * @param {string} message - the message to send.
     * @param {string} encoding - the message encoding type.
     * @param {boolean} withResponse - if true, resolve after peripheral's response.
     * @return {Promise} - a promise from the remote send request.
     */
    write (serviceId, characteristicId, message, encoding = null, withResponse = null) {
        if (encoding) {
            params.encoding = encoding;
        }
        if (withResponse) {
            params.withResponse = withResponse;
        }
        messageUint8 = new Uint8Array(message);
        //console.log(messageUint8);
        return this._server.getPrimaryService(serviceId)
				.then(service => service.getCharacteristic(characteristicId))
				.then(characteristic => characteristic.writeValue(messageUint8))
				.catch(e => {
					this._sendDisconnectError(e);
				});
    }
    



    _sendRequestError ( e ) {
        // log.error(`BLE error: ${JSON.stringify(e)}`);
      console.log(e);
      //   this._runtime.emit(this._runtime.constructor.PERIPHERAL_REQUEST_ERROR, {
      //      message: `Scratch lost connection to`,
       //     extensionId: this._extensionId
        //});
    }
    
   _ConnectionError (e) {
        // log.error(`BLE error: ${JSON.stringify(e)}`);
        console.log('BLE error : %s',e);

        this._connected = false;

        this._runtime.emit(this._runtime.constructor.PERIPHERAL_DISCONNECT_ERROR, {
            message: `Bluetooth connection error to`,
            extensionId: this._extensionId
        });
    }

    _sendDisconnectError (e) {
        // log.error(`BLE error: ${JSON.stringify(e)}`);
        console.log('BLE error : %s',e);
        if (e.type === "gattserverdisconnected")
        {
			this.parentBLE._runtime.emit(this.parentBLE._runtime.constructor.PERIPHERAL_DISCONNECT_ERROR, {
				message: `Scratch lost connection to`,
				extensionId: this.parentBLE._extensionId
			});
		}
    }

    _sendDiscoverTimeout () {
        if (this._discoverTimeoutID) {
            window.clearTimeout(this._discoverTimeoutID);
        }
        this._runtime.emit(this._runtime.constructor.PERIPHERAL_SCAN_TIMEOUT);
    }
};

module.exports = WEBBLE;
