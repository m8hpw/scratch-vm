const ArgumentType = require('../../extension-support/argument-type');
const BlockType = require('../../extension-support/block-type');
const Cast = require('../../util/cast');
const formatMessage = require('format-message');
const color = require('../../util/color');
const BLE = require('../../io/webble');
const Base64Util = require('../../util/base64-util');
const MathUtil = require('../../util/math-util');
const RateLimiter = require('../../util/rateLimiter.js');
const log = require('../../util/log');
const EventEmitter = require( 'events' );


/**
 * Icon svg to be displayed at the left edge of each extension block, encoded as a data URI.
 * @type {string}
 */
// eslint-disable-next-line max-len
const iconURI = 'data:image/png;base64,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';


/**
 * A list of WeDo 2.0 BLE characteristic UUIDs.
 *
 * Characteristics on DEVICE_SERVICE:
 * - ATTACHED_IO
 *
 * Characteristics on IO_SERVICE:
 * - INPUT_VALUES
 * - INPUT_COMMAND
 * - OUTPUT_COMMAND
 *
 * @enum
 */

//MOVE_HUB_HARDWARE_HANDLE = 0x0E
const HUB_HW_UUID_SERV = '00001623-1212-efde-1623-785feabcd123';
const HUB_HW_UUID_CHAR = '00001624-1212-efde-1623-785feabcd123';

const PACKET_VER = 0x01;
const LEGO_MOVE_HUB = "LEGO Move Hub";

// PORTS
const BoostPort = {
	C : 0x01,
	D : 0x02,
	LED : 0x32,
	A : 0x37,
	B : 0x38,
	AB : 0x39,
	TILT_SENSOR : 0x3A,
	AMPERAGE : 0x3B,
	VOLTAGE : 0x3C,
};

const BoostPortNames = {
	0x01 : "C",
	0x02 : "D",
	0x32 : "LED",
	0x37 : "A",
	0x38 : "B",
	0x39 : "AB",
	0x3A : "TILT",
	0x3B : "AMPERAGE",
	0x3C : "VOLTAGE",
};



const BoostMsgTypes={
	DEVICE_INFO : 0x01,
	// 0501010305 gives 090001030600000010
	DEVICE_SHUTDOWN : 0x02,  // sent when hub shuts down by button hold
	PING_RESPONSE : 0x03,
	PORT_INFO : 0x04,
	PORT_CMD_ERROR : 0x05,
	SET_PORT_VAL : 0x81,
	PORT_STATUS : 0x82,
	SENSOR_SUBSCRIBE : 0x41,
	SENSOR_SOMETHING1 : 0x42,  // it is seen close to sensor subscribe commands. Subscription options? Initial value?
	SENSOR_DATA : 0x45,
	SENSOR_SUBSCRIBE_ACK : 0x47,
};

const BoostDeviceTypes = {
    0x27 : "MOTOR" ,
    0x17 : "LED" ,
    0x28 : "TILT",
    0x25 : "DISTANCE",
    0x14 : "VOLTAGE",
    0x15 : "AMPERAGE" ,
    0x26 : "IMOTOR",
};


// NOTIFICATIONS
const BoostStatus = {
	STARTED : 0x01,
	CONFLICT : 0x05,
	FINISHED : 0x0a,
	INPROGRESS : 0x0c,  // FIXME: not sure about description
	INTERRUPTED : 0x0e,  // FIXME:  not sure about description
};
// COLORS
const BoostColor = {
BLACK : 0x00,
PINK : 0x01,
PURPLE : 0x02,
BLUE : 0x03,
LIGHTBLUE : 0x04,
CYAN : 0x05,
GREEN : 0x06,
YELLOW : 0x07,
ORANGE : 0x09,
RED : 0x09,
WHITE : 0x0a,
NONE : 0xFF,
};
// Info 
const BoostInfo = {
DEVICE_NAME : 0x01,
BUTTON_STATE : 0x02,
FIRMWARE_VERSION : 0x03,
SOME4 : 0x04,
SOME5_JITTERING : 0x05,
SOME6 : 0x06,
SOME7 : 0x07,
MANUFACTURER : 0x08,
HW_VERSION : 0x09,
SOME10 : 0x0a,
SOME11 : 0x0b,
SOME12 : 0x0c
};
const BoostInfoAction = {
SUBSCRIBE : 0x02,
UNSUBSCRIBE : 0x03,
GET : 0x05,
};

const BoostTiltSensorMode = {

    TWO_AXIS_FULL : 0x00,
    TWO_AXIS_SIMPLE : 0x01,
    THREE_AXIS_SIMPLE : 0x02,
    BUMP_COUNT : 0x03,
    THREE_AXIS_FULL : 0x04,
};

const BoostTiltSensorTRI = { 
    BACK : 0x00,
    UP : 0x01,
    DOWN : 0x02,
    LEFT : 0x03,
    RIGHT : 0x04,
    FRONT : 0x05,
};
const BoostTiltSensorDUO = { 
    HORIZ : 0x00,
    DOWN : 0x03,
    LEFT : 0x05,
    RIGHT : 0x07,
    UP : 0x09,
};

const BoostMotorsConstants = { 
    TRAILER : [0x64,0x7f,0x03],  // NOTE: \x64 is 100, might mean something, also trailer might be a sequence terminator
    // TODO: investigate sequence behavior, seen with zero values passed to angled mode
    //trailer is not required for all movement types
    MOVEMENT_TYPE : 0x11,

    CONSTANT_SINGLE : 0x01,
    CONSTANT_GROUP : 0x02,
    SOME_SINGLE : 0x07,
    SOME_GROUP : 0x08,
    TIMED_SINGLE : 0x09,
    TIMED_GROUP : 0x0A,
    ANGLED_SINGLE : 0x0B,
    ANGLED_GROUP : 0x0C,

    // MOTORS
    SENSOR_SOMETHING1 : 0x00,  // TODO: understand it
    SENSOR_SPEED : 0x01,
    SENSOR_ANGLE : 0x02,
};

const   BoostDevStatus ={
	DETACHED : 0x00,
    DEVICE : 0x01,
    GROUP : 0x02,
}


const BoostColorDistanceSensor = {
    COLOR_ONLY : 0x00,
    DISTANCE_INCHES : 0x01,
    COUNT_2INCH : 0x02,
    DISTANCE_HOW_CLOSE : 0x03,
    DISTANCE_SUBINCH_HOW_CLOSE : 0x04,
    OFF1 : 0x05,
    STREAM_3_VALUES : 0x06,
    OFF2 : 0x07,
    COLOR_DISTANCE_FLOAT : 0x08,
    LUMINOSITY : 0x09,
    SOME_20BYTES : 0x0a,  // TODO: understand it
}
/**
 * A time interval to wait (in milliseconds) while a block that sends a BLE message is running.
 * @type {number}
 */
const BLESendInterval = 100;

/**
 * A maximum number of BLE message sends per second, to be enforced by the rate limiter.
 * @type {number}
 */
const BLESendRateMax = 20;

class Peripheral
{
	 constructor( parent, port)
	 {
        this.parent = parent;
        this.port = port;
	}


      _write_to_hub(msg_type, params)
        {
			cmd = [this.port].concat(params);
			this.parent.send(msg_type, cmd);
		}
	  finished()
	  {
		this._working = false;
	  }	
	  started()
	  {
        this._working = true;
	  }

     _port_subscribe(mode, granularity, enable)
     {
        let params = [mode].concat(this._convertToLittleEnd(granularity,2),[0,0],enable?1:0);
        this._write_to_hub(BoostMsgTypes.SENSOR_SUBSCRIBE, params);
     }
     
     _convertFromLittleEnd(data)
	{
		out = 0;
		mult = 1;
		for(i=0;i<data.length;i++)
		{
			out+= data[i] * (mult);
			mult*=256;
		}
		return(out)
	}

	_convertToLittleEnd(data,len)
	{
			out = Array(len);
			mult = 1;
			for(i=0;i<((len>3)?3:len);i++)
			{
				out[i] = parseInt((data/mult))%256;
				mult*=256;
			}
			if (len == 4) out[3]=0;
			return out;
	}


}

class Sensor extends Peripheral
{
	constructor(parent,port)
	{
		super(parent,port);
	}
	subscribe(mode, granularity=1, async=false)
	{
        this._port_subscription_mode = mode;
        this._port_subscribe(this._port_subscription_mode, granularity, true);
        this.isSubcribed = true;
    }
    
    unsubscribe(async=false)
    {
       if (this._port_subscription_mode==null)
       {
            console.log("Attempt to unsubscribe while never subscribed.");
            return;
       }
		this._port_subscribe(this._port_subscription_mode, 0, false);
        this._port_subscription_mode = null;
		this.isSubcribed = true;

    }
}

class OutputDevice extends Peripheral
{
	constructor(parent,port)
	{
		super(parent,port);
        this._working = false;     
        this.eventEmitter = new EventEmitter();
    }


    finished()
    {
		if (this._working)
			this.eventEmitter.emit('finished');
    }
    in_progress()
    {
        return this._working;
    }
}

class BoostEncodedMotor extends OutputDevice
{
	constructor (parent,port)
	{
		super(parent,port);
	} 
	
	finished()
    {
		super.finished();
		if (this._working)
			this.eventEmitter.emit('finished');
    }
    
    _speed_abs(relative){
        if (relative < -100)
            //log.warning("Speed cannot be less than -1")
            relative = -100;

        if (relative > 100)
            //log.warning("Speed cannot be more than 1")
            relative = 100;

        return relative>=0?relative:(127+relative+128);
	}
	
    _wrap_and_write(mtype, params, speed_primary, speed_secondary)
    {
        if (this.port == BoostPort.AB)
            mtype += 1;  // de-facto rule

        abs_primary = this._speed_abs(speed_primary);
        abs_secondary = this._speed_abs(speed_secondary);

        if ((mtype == BoostMotorsConstants.ANGLED_GROUP) && (!abs_secondary || !abs_primary))
            return;//raise ValueError("Cannot have zero speed in double angled mode")  // otherwise it gets nuts

        params = [BoostMotorsConstants.MOVEMENT_TYPE, mtype].concat(params);
        params = params.concat(abs_primary);
        
        if (this.port == BoostPort.AB)
            params = params.concat(abs_secondary);

        params = params.concat(BoostMotorsConstants.TRAILER);

        this._write_to_hub(BoostMsgTypes.SET_PORT_VAL, params)
    }

    dtimed(seconds, speed_primary=1, speed_secondary=null, async=false)
    {
		if (this.in_progress())
				this.stop();
	
		if (speed_secondary == null)
			speed_secondary = speed_primary;

		milliseconds =  parseInt(seconds * 1000);
		params = this._convertToLittleEnd(milliseconds,2); //<H

		this.started();
		this._wrap_and_write(BoostMotorsConstants.TIMED_SINGLE, params, speed_primary, speed_secondary);

        return new Promise(resolve => this.eventEmitter.on('finished',resolve));
    }

    angled(angle, speed_primary=1, speed_secondary=null, async=false)
    {
        if (speed_secondary==null)
            speed_secondary = speed_primary;

        angle = parseInt(angle);
        if (angle < 0)
        {
            angle = -angle;
            speed_primary = -speed_primary;
            speed_secondary = -speed_secondary;
		}
        params = this._convertToLittleEnd(angle,4);
        
        this.started();
        this._wrap_and_write(BoostMotorsConstants.ANGLED_SINGLE, params, speed_primary, speed_secondary);
        return new Promise(resolve => this.eventEmitter.on('finished',resolve));
    }

    constant(speed_primary=1, speed_secondary=null, async=false)
    {
        if (speed_secondary == null)
            speed_secondary = speed_primary;

        this.started();
        this._wrap_and_write(BoostMotorsConstants.CONSTANT_SINGLE, [], speed_primary, speed_secondary);
    }

    __some(speed_primary=1, speed_secondary=null, async=false)
    {
        if (speed_secondary == null)
            speed_secondary = speed_primary;

        this.started();
        this._wrap_and_write(BoostMotorsConstants.SOME_SINGLE, [], speed_primary, speed_secondary);
        this._wait_sync(async);
    }

    stop(async=false)
    {
        if (this.in_progress())
			this.finished();

        this.constant(0, async=async);
			
        return;
	}
	
    handle_port_data(data)
    {
        if (this._port_subscription_mode == BoostMotorsConstants.SENSOR_ANGLE)
        {
            rotation = this._convertFromLittleEnd(data.slice(4,9));
            this._notify_subscribers(rotation)
		}
        else if (this._port_subscription_mode == BoostMotorsConstants.SENSOR_SOMETHING1)
        {
            // TODO: understand what it means
            //rotation = usbyte(data, 4)
            //this._notify_subscribers(rotation)
        }
        else if (this._port_subscription_mode == BoostMotorsConstants.SENSOR_SPEED)
        {
            rotation = data[4];
            this._notify_subscribers(rotation)
		}
        else
            console.log("Got motor sensor data while in unexpected mode: %s", this._port_subscription_mode);
     }
}

class BoostLED extends OutputDevice
{
	constructor (parent,port)
	{
		super(parent,port);
	} 
	
	setLED (hue) {
		this._write_to_hub(BoostMsgTypes.SET_PORT_VAL,[0x01,0x51,0x00,hue]);
    }


    /**
     * Switch off the LED on the WeDo 2.0.
     * @return {Promise} - a promise of the completion of the stop led send operation.
     */
    stopLED () {
        setLED(0);
    }

}




 
class TiltSensor extends Sensor
{
	constructor(parent,port)
    {
		super(parent,port);
	}
    subscribe(mode = BoostTiltSensorMode.THREE_AXIS_FULL)
    {
		super.subscribe(mode, granularity=1);
	}

    handle_port_data(data){
        switch(this._port_subscription_mode) {
		case  BoostTiltSensorMode.THREE_AXIS_SIMPLE:
            this.state = data[4];
            break;
        case  BoostTiltSensorMode.TWO_AXIS_SIMPLE:
            this.state = data[4];
            break;
        case  BoostTiltSensorMode.MODE_BUMP_COUNT:
            this.bump_count = this._convertFromLittleEnd(data.slice(4,6));
            break;
        case  BoostTiltSensorMode.TWO_AXIS_FULL:
            this.roll = this._byte2deg(data[4]);
            this.pitch = this._byte2deg(data[5]);
            break;
        case  BoostTiltSensorMode.THREE_AXIS_FULL:
            this.roll = this._byte2deg(data[4]);
            this.pitch = this._byte2deg(data[5]);
            this.yaw = this._byte2deg(data[6]);// did I get the order right?
            break;
        default:
            console.log("Got tilt sensor data while in unexpected mode: %s", this._port_subscription_mode)
		}
	}
    _byte2deg(val)
    {
        if (val > 90)
            return val - 256;
        else
            return val;
     }
}


class ColorDistanceSensor extends Sensor
{
    constructor(parent,port)
    {
		super(parent,port);
	}
    subscribe()
    {
		super.subscribe(BoostColorDistanceSensor.COLOR_DISTANCE_FLOAT, granularity=1);
	}
    handle_port_data(data)
    {
        switch (this._port_subscription_mode) {
          case BoostColorDistanceSensor.COLOR_DISTANCE_FLOAT:
            this.color = data[4];
            this.distance = data[5];
            partial = data[7];
            if (partial) this.distance += 1.0 / partial;
            break;
         case BoostColorDistanceSensor.COLOR_ONLY:
            this.color = data[4];
            break;
         case BoostColorDistanceSensor.DISTANCE_INCHES:
            this.distance = data[4];
            break;
        case BoostColorDistanceSensor.DISTANCE_HOW_CLOSE:
            this.distance = data[4];
            break;
        case BoostColorDistanceSensor.DISTANCE_SUBINCH_HOW_CLOSE:
            this.distance = data[4];
            break;
        case BoostColorDistanceSensor.OFF1:
        case BoostColorDistanceSensor.OFF2:
            console.log("Turned off led on %s", this);
            break;
        case BoostColorDistanceSensor.COUNT_2INCH:
            let count = this._convertFromLittleEnd(data.slice(4,8));
            break;
        case BoostColorDistanceSensor.STREAM_3_VALUES:
            // TODO: understand better meaning of these 3 values
            val1 = data[4];
            val2 = data[6];
            val3 = data[8];
            break;
        case BoostColorDistanceSensor.LUMINOSITY:
            this.luminosity = this._convertFromLittleEnd(data.slice(4,6)); // 1023.0;
            break;
        default:  // TODO: support whatever we forgot
            console.log("Unhandled data in mode %s: %s", this._port_subscription_mode, str2hex(data));
		};
	}
}


class VoltageSensor extends Sensor
{
    constructor(parent,port)
    {
		super(parent,port);
	}
    subscribe()
    {
		super.subscribe(0x01, granularity=1);
	}
    handle_port_data(data)
    {
		  this.value = this._convertFromLittleEnd(data.slice(4,6))/4096.0;
	}
}

class AmperageSensor extends Sensor
{
    constructor(parent,port)
    {
		super(parent,port);
	}
    subscribe()
    {
		super.subscribe(0x01, granularity=1);
	}
    handle_port_data(data)
    {
		  this.value = this._convertFromLittleEnd(data.slice(4,6))/4096.0;
	}
}



class Boost {

    constructor (runtime, extensionId) {
        /**
         * The Scratch 3.0 runtime used to trigger the green flag button.
         * @type {Runtime}
         * @private
         */
        this._runtime = runtime;
        this._runtime.on('PROJECT_STOP_ALL', this.stopAll.bind(this));

        /**
         * The id of the extension this peripheral belongs to.
         */
        this._extensionId = extensionId;

        this._ble = null;
        this._runtime.registerPeripheralExtension(extensionId, this);
        
        this.devices = [];
        this.color_distance_sensor_port = null;        
    }


    connect (id) {
        if (this._ble) {
            this._ble.connectPeripheral(id);
        }
    }

    disconnect () {

        if (this._ble) {
            this._ble.disconnect();
        }
    }
    /**
     * Stop the tone playing and motors on the WeDo 2.0 peripheral.
     */
    stopAll () {
        if (!this.isConnected()) return;
        //this.stopAllMotors();
    }
    
          /**
     * Called by the runtime when user wants to scan for a WeDo 2.0 peripheral.
     */  
    scan () {
        if (this._ble) {
            this._ble.disconnect();
        }
        this._ble = new BLE(this._runtime, 
                            this._extensionId,
                            HUB_HW_UUID_CHAR,
                             HUB_HW_UUID_SERV,
                             this._onConnect,this);
    }
    /**
     * Called by the runtime to detect whether the WeDo 2.0 peripheral is connected.
     * @return {boolean} - the connected state.
     */
    isConnected () {
        let connected = false;
        if (this._ble) {
            connected = this._ble.isConnected();
        }
        return connected;
    }

    /**
     * Write a message to the WeDo 2.0 peripheral BLE socket.
     * @param {number} uuid - the UUID of the characteristic to write to
     * @param {Array} message - the message to write.
     * @param {boolean} [useLimiter=true] - if true, use the rate limiter
     * @return {Promise} - a promise result of the write operation
     */
    send (commandID, values, useLimiter = true) {
        if (!this.isConnected()) return Promise.resolve();

        message = [values.length + 3,PACKET_VER, commandID].concat(values);
        
        return this._ble.write(
            HUB_HW_UUID_SERV,
            HUB_HW_UUID_CHAR,
            message
            );
    }
    
	_attach_device(dev_type, port)
	{
		if ((port in BoostPortNames) && (dev_type in BoostDeviceTypes))
			console.log("Attached %s on port %s", BoostDeviceTypes[dev_type], BoostPortNames[port])
		else
			console.log("Attached device 0x%x on port 0x%x", dev_type, port)

        
		switch(dev_type){
		
			
			case 0x27: //MOTOR
			case 0x26: //IMOTOR
				this.devices[BoostPortNames[port]] = new BoostEncodedMotor(this,port);
				break;
				//this.motor_external = EncodedMotor(port);
				//this.devices[BoostPortNames[port]] = self.motor_external;
				break;
			case 0x25: //DISTANCE
				//this.color_distance_sensor = ColorDistanceSensor(port);
				this.color_distance_sensor_port = port;
				this.devices["DISTANCE"] =  new ColorDistanceSensor(this,port);
				break;
			case 0x17: //LED
				this.devices[BoostPortNames[port]] = new BoostLED(this,port);
				break;
			case 0x28: //TILT
				this.devices[BoostPortNames[port]] = new TiltSensor(this,port);
				break;
			case 0x15: //AMPERAGE
				this.devices[BoostPortNames[port]] = new AmperageSensor(this,port);
				break;
			case 0x14: //VOLTAGE
			 	this.devices[BoostPortNames[port]] = new VoltageSensor(this,port)
				break;
			default:
				console.log("Unhandled peripheral type 0x%x on port 0x%x", dev_type, port);
				//self.devices[port] = Peripheral(port);
		}
	}
    
    _handle_device_info(data)
    {
        kind = data[3];
        if (kind == 2)
            this.button.handle_port_data(data);

        if (data[4] == 0x06)
            this.info[kind] = data.slice(5,data.length);
        else
            console.log("Unhandled device info: %s", str2hex(data));
    }

    _handle_sensor_data(data)
    {
        portName = this.portIDtoName(data[3]);
        if (typeof this.devices[portName]  === 'undefined')
        {
            console.log("Notification on port with no device: %s", portName);
            return;
        }
        this.devices[portName].handle_port_data(data)
    }

    _handle_port_status(data)
    {
        portName = this.portIDtoName(data[3]);
        let portStatus = data[4];
        switch (portStatus) {
			case BoostStatus.STARTED:
				this.devices[portName].started();
				break;
			case BoostStatus.FINISHED:
				this.devices[portName].finished();
				break;
			case BoostStatus.CONFLICT:
				console.log("Command conflict on port %s", portName);
				this.devices[portName].finished();
				break;
            case BoostStatus.INPROGRESS:
				console.log("Another command is in progress on port %s", portName);
				this.devices[portName].finished();
				break;
            case BoostStatus.INTERRUPTED:
				console.log("Command interrupted on port %s",portName);
				this.devices[portName].finished();
				break;
			default:
				console.log("Unhandled status value: %s on port %s", status.toString(16), portName);
		};
	}

    _handle_port_info(data)
    {
		portID = data[3];
        portName = this.portIDtoName(portID);
        portStatus = data[4];

        switch(portStatus) {
			case BoostDevStatus.DETACHED:
            console.log("Detached %s", portName);
            break;
        case BoostDevStatus.DEVICE:
        case BoostDevStatus.GROUP:
            dev_type = data[5];
            this._attach_device(dev_type, portID);
            break;
        default:
            console.log("Unhandled device status: %s", status)
		}
	}
	
	portIDtoName(port)
	{
		return (this.color_distance_sensor_port==port)?"DISTANCE":BoostPortNames[port];
	}

    /**
     * Process the sensor data from the incoming BLE characteristic.
     * @param {object} base64 - the incoming BLE data.
     * @private
     */
    onMessage (value) {
		data = new Uint8Array(value.byteLength);
		for(i=0;i<value.byteLength;i++)
			data[i]=value.getUint8(i);
        /**
         * If first byte of data is '1' or '2', then either clear the
         * sensor present in ports 1 or 2 or set their format.
         *
         * If first byte of data is anything else, read incoming sensor value.
         */
        switch (data[2]) {
		case  BoostMsgTypes.PORT_INFO:
            this._handle_port_info(data);
            break;
        case BoostMsgTypes.PORT_STATUS:
            this._handle_port_status(data);
            break;
        case BoostMsgTypes.SENSOR_DATA:
            this._handle_sensor_data(data);
            break;
        case  BoostMsgTypes.SENSOR_SUBSCRIBE_ACK:
            port = this.portIDtoName(data[3]);
            console.log("Sensor subscribe ack on port %s", port);
            this.devices[port].started();
            break;
        case  BoostMsgTypes.PORT_CMD_ERROR:
            console.log("Command error: %s", data.slide(3,data.length));
            port = this.portIDtoName(data[3]);
            this.devices[port].finished();
            break;
        case BoostMsgTypes.DEVICE_SHUTDOWN:
            console.log("Device reported shutdown: %s", str2hex(data));
            break;
        case BoostMsgTypes.DEVICE_INFO:
             this._handle_device_info(data);
             console.log("Unhandled msg type 0x%x: %s", msg_type, str2hex(orig));
        }
    }

    /**
     * Clear the sensor or motor present at port 1 or 2.
     * @param {number} connectID - the port to clear.
     * @private
     */
    _clearPort (connectID) {
    }
}





/**
 * Scratch 3.0 blocks to interact with a LEGO WeDo 2.0 peripheral.
 */
class Scratch3BoostBlocks {

    static get EXTENSION_ID () {
        return 'boost';
    }

    constructor (runtime) {
        /**
         * The Scratch 3.0 runtime.
         * @type {Runtime}
         */
        this.runtime = runtime;
        // Create a new WeDo 2.0 peripheral instance
        this._peripheral = new Boost(this.runtime, Scratch3BoostBlocks.EXTENSION_ID);
         
    }

    /**
     * @returns {object} metadata for this extension and its blocks.
     */
    getInfo () {
        return {
            id: Scratch3BoostBlocks.EXTENSION_ID,
            name: 'Boost',
            blockIconURI: iconURI,
            showStatusButton: true,
            blocks: [
            {
                    opcode: 'motorOn',
                    text: formatMessage({
                        id: 'boost.motorOn',
                        default: 'turn  [MOTOR_ID] at speed [SPEED]',
                        description: 'turn a motor on'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        MOTOR_ID: {
                            type: ArgumentType.STRING,
                            menu: 'MOTOR_ID',
                            defaultValue: 'A'
                        },
                        SPEED: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        }
                    }
                },
                {
                    opcode: 'motorOnForDuration',
                    text: formatMessage({
                        id: 'boost.motorOnForDuration',
                        default: 'turn  [MOTOR_ID] for [DURATION] s at speed [SPEED]',
                        description: 'turn a motor on for some time'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        MOTOR_ID: {
                            type: ArgumentType.STRING,
                            menu: 'MOTOR_ID',
                            defaultValue: 'A'
                        },
                        DURATION: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        },
                        SPEED: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        }
                    }
                },
                {
                    opcode: 'motorOnForAngle',
                    text: formatMessage({
                        id: 'boost.motorOnForAngle',
                        default: 'turn [MOTOR_ID] by [ANGLE] degrees at speed [SPEED]',
                        description: 'turn a motor on for some angle'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        MOTOR_ID: {
                            type: ArgumentType.STRING,
                            menu: 'MOTOR_ID',
                            defaultValue: 'A'
                        },
                        ANGLE: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        },
                        SPEED: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        }
                    }
                },
                 {
                    opcode: 'motorsABOn',
                    text: formatMessage({
                        id: 'boost.motorABOn',
                        default: 'turn AB at speed [SPEEDA] and [SPEEDB]',
                        description: 'turn motors A and B on'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        SPEEDA: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        },
                        SPEEDB: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        }
                    }
                },
                {
                    opcode: 'motorsABOnForDuration',
                    text: formatMessage({
                        id: 'boost.motorABOnForDuration',
                        default: 'turn AB for [DURATION] s at speed [SPEEDA] and [SPEEDB]',
                        description: 'turn motors A and B on for some time'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        DURATION: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        },
                        SPEEDA: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        },
                        SPEEDB: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        }
                    }
                },
                {
                    opcode: 'motorsABOnForAngle',
                    text: formatMessage({
                        id: 'boost.motorABOnForAngle',
                        default: 'turn AB by [ANGLE] degrees at speed [SPEEDA] and [SPEEDB]',
                        description: 'turn motors A and B on for some angles'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        ANGLE: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        },
                        SPEEDA: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        },
                        SPEEDB: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 1
                        }
                    }
                },

                {
                    opcode: 'motorOff',
                    text: formatMessage({
                        id: 'boost.motorOff',
                        default: 'turn [MOTOR_ID] off',
                        description: 'turn a motor off'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        MOTOR_ID: {
                            type: ArgumentType.STRING,
                            menu: 'MOTOR_ID',
                            defaultValue: 'AB'
                        }
                    }
                },
                {
                    opcode: 'setLightHue',
                    text: formatMessage({
                        id: 'boost.setLightHue',
                        default: 'set light color to [HUE]',
                        description: 'set the LED color'
                    }),
                    blockType: BlockType.COMMAND,
                    arguments: {
                        HUE: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 50
                        }
                    }
                },
                {
                    opcode: 'whenOpticSensor',
                    text: formatMessage({
                        id: 'boost.whenOpticSensor',
                        default: 'when [SENSOR] [OP] [REFERENCE]',
                        description: 'check for when sensor is < or > than reference'
                    }),
                    blockType: BlockType.HAT,
                    arguments: {
						SENSOR: {
			                type: ArgumentType.STRING,
                            menu: 'OPTIC_SENSOR_ID',
                            defaultValue: 'distance'
                        },				
                        OP: {
                            type: ArgumentType.STRING,
                            menu: 'OP',
                            defaultValue: '<'
                        },
                        REFERENCE: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 50
                        }
                    }
                },
                {
                opcode: 'getOpticSensor',
                    text: formatMessage({
                        id: 'boost.getOpticSensor',
                        default: '[SENSOR]',
                        description: 'the value returned by the distance sensor'
                    }),
                    blockType: BlockType.REPORTER,
                      arguments: {
						SENSOR: {
			                type: ArgumentType.STRING,
                            menu: 'OPTIC_SENSOR_ID',
                            defaultValue: 'distance'
                        },
                      }
                },
                {
                    opcode: 'whenTiltSensor',
                    text: formatMessage({
                        id: 'boost.whenTiltSensor',
                        default: 'when [SENSOR] [OP] [REFERENCE]',
                        description: 'check for when sensor is < or > than reference'
                    }),
                    blockType: BlockType.HAT,
                    arguments: {
						SENSOR: {
			                type: ArgumentType.STRING,
                            menu: 'TILT_SENSOR_ID',
                            defaultValue: 'roll'
                        },				
                        OP: {
                            type: ArgumentType.STRING,
                            menu: 'OP',
                            defaultValue: '<'
                        },
                        REFERENCE: {
                            type: ArgumentType.NUMBER,
                            defaultValue: 50
                        }
                    }
                },
                {
                opcode: 'getTiltSensor',
                    text: formatMessage({
                        id: 'boost.getTiltSensor',
                        default: '[SENSOR]',
                        description: 'the value returned by the tilt sensor'
                    }),
                    blockType: BlockType.REPORTER,
                      arguments: {
						SENSOR: {
			                type: ArgumentType.STRING,
                            menu: 'TILT_SENSOR_ID',
                            defaultValue: 'roll'
                        },
                      }
                },
                                {
                opcode: 'getElectricalSensor',
                    text: formatMessage({
                        id: 'boost.getElectricalSensor',
                        default: '[SENSOR]',
                        description: 'the value returned by the electrical sensor'
                    }),
                    blockType: BlockType.REPORTER,
                      arguments: {
						SENSOR: {
			                type: ArgumentType.STRING,
                            menu: 'ELECTRICAL_SENSOR_ID',
                            defaultValue: 'voltage'
                        },
                      }
                },

            ],
            menus: {
                MOTOR_ID: [
                    {
                        text: formatMessage({
                            id: 'boost.motorId.ab',
                            default: 'motor A and B',
                            description: 'label for motor A and B element in motor menu for LEGO boost extension'
                        }),
                        value: "AB"
                    },
                    {
                        text: formatMessage({
                            id: 'boost.motorId.a',
                            default: 'A',
                            description: 'label for motor A element in motor menu for LEGO boost extension'
                        }),
                        value: "A"
                    },
                    {
                        text: formatMessage({
                            id: 'boost.motorId.b',
                            default: 'B',
                            description: 'label for motor B element in motor menu for LEGO boost extension'
                        }),
                        value: "B"
                    },
                                        {
                        text: formatMessage({
                            id: 'boost.motorId.c',
                            default: 'C',
                            description: 'label for motor C element in motor menu for LEGO boost extension'
                        }),
                        value: "C"
                    },
                    {
                        text: formatMessage({
                            id: 'boost.motorId.d',
                            default: 'D',
                            description: 'label for motor D element in motor menu for LEGO boost extension'
                        }),
                        value: "D"
                    },
                ],
                OPTIC_SENSOR_ID: [
                   {
                        text: formatMessage({
                            id: 'boost.opticSensor.distance',
                            default: 'distance',
                            description: 'label for distance in optic detector menu for LEGO boost extension'
                        }),
                        value: "distance"
                    },
                    {
                        text: formatMessage({
                            id: 'boost.opticSensor.color',
                            default: 'color',
                            description: 'label for color in optic detector menu for LEGO boost extension'
                        }),
                        value: "color"
                    },
                ],
                TILT_SENSOR_ID: [
                    {
                        text: formatMessage({
                            id: 'boost.tiltSensor.roll',
                            default: 'roll',
                            description: 'label for roll detector menu for LEGO boost extension'
                        }),
                        value: "roll"
                    },
                    {
                        text: formatMessage({
                            id: 'boost.tiltSensor.pitch',
                            default: 'pitch',
                            description: 'label for pitch detector menu for LEGO boost extension'
                        }),
                        value: "pitch"
                    },
                     {
                        text: formatMessage({
                            id: 'boost.tiltSensor.yaw',
                            default: 'yaw',
                            description: 'label for yaw detector menu for LEGO boost extension'
                        }),
                        value: "yaw"
                    },
                ],
                ELECTRICAL_SENSOR_ID: [
                    {
                        text: formatMessage({
                            id: 'boost.electricalSensor.voltage',
                            default: 'voltage',
                            description: 'label for voltage detector menu for LEGO boost extension'
                        }),
                        value: "voltage"
                    },
                    {
                        text: formatMessage({
                            id: 'boost.electricalSensor.amperage',
                            default: 'amperage',
                            description: 'label for amperage detector menu for LEGO boost extension'
                        }),
                        value: "amperage"
                    },
                ],

                OP: ['<', '>'],
            }
        };
    }

	motorOn (args) {
	// TODO: cast args.MOTOR_ID?
		let speed = Cast.toNumber(args.SPEED);
		return this._peripheral.devices[args.MOTOR_ID].constant(speed);
	}
	motorOnForDuration (args) {
	// TODO: cast args.MOTOR_ID?
		let speed = Cast.toNumber(args.SPEED);
		let durationMS = Cast.toNumber(args.DURATION);
		durationMS = MathUtil.clamp(durationMS, 0, 15000);
		return this._peripheral.devices[args.MOTOR_ID].dtimed(durationMS,speed);
	}
	motorOnForAngle (args) {
		// TODO: cast args.MOTOR_ID?
		let speed = Cast.toNumber(args.SPEED);
		let angle = Cast.toNumber(args.ANGLE);
		angle = MathUtil.clamp(angle, -3600, 3600);
		return this._peripheral.devices[args.MOTOR_ID].angled(angle,speed);
	}

	motorsABOn (args) {
	// TODO: cast args.MOTOR_ID?
		let speedA = Cast.toNumber(args.SPEEDA);
		let speedB = Cast.toNumber(args.SPEEDB);
		return this._peripheral.devices['AB'].constant(speedA,speedB);
	}
	motorsABOnForDuration (args) {
		// TODO: cast args.MOTOR_ID?
		let speedA = Cast.toNumber(args.SPEEDA);
		let speedB = Cast.toNumber(args.SPEEDB);
		let durationMS = Cast.toNumber(args.DURATION);
		durationMS = MathUtil.clamp(durationMS, 0, 15000);
		return this._peripheral.devices['AB'].dtimed(durationMS,speedA,speedB);
	}
	motorsABOnForAngle (args) {
		// TODO: cast args.MOTOR_ID?
		let speedA = Cast.toNumber(args.SPEEDA);
		let speedB = Cast.toNumber(args.SPEEDB);
		let angle = Cast.toNumber(args.ANGLE);
		angle = MathUtil.clamp(angle, -3600, 3600);
		return this._peripheral.devices['AB'].angled(angle,speedA,speedB);
	}

	motorOff (args) {
		// TODO: cast args.MOTOR_ID?
		return this._peripheral.devices[args.MOTOR_ID].stop();
		//setTimeout(resolve, durationMS);
	}
    /**
     * Set the LED's hue.
     * @param {object} args - the block's arguments.
     * @property {number} HUE - the hue to set, in the range [0,100].
     * @return {Promise} - a Promise that resolves after some delay.
     */
    setLightHue (args) {
        // Convert from [0,100] to [0,10]
        let inputHue = Cast.toNumber(args.HUE);
        inputHue = MathUtil.wrapClamp(inputHue, 0, 100);
        const hue = parseInt(inputHue * 9 / 100);
        this._peripheral.devices['LED'].setLED(hue);

        return new Promise(resolve => {
            window.setTimeout(() => {
                resolve();
            }, BLESendInterval);
        });
    }
    
    whenOpticSensor (args) {
		if (!("DISTANCE" in this._peripheral.devices)) return false;
		if  (!this._peripheral.devices["DISTANCE"].isSubcribed) this._peripheral.devices["DISTANCE"].subscribe();
		switch (args.SENSOR) {
		case 'distance':
			value = this._peripheral.devices["DISTANCE"].distance;
			break;
		case 'color':
			value = this._peripheral.devices["DISTANCE"].color;
		}
        switch (args.OP) {
        case '<':
            return value < Cast.toNumber(args.REFERENCE);
        case '>':
            return value > Cast.toNumber(args.REFERENCE);
        default:
            console.log(`Unknown comparison operator in whenDistance: ${args.OP}`);
            return false;
        }
    }
    getOpticSensor (args) {
		if (!("DISTANCE" in this._peripheral.devices)) return 0;
		if  (!this._peripheral.devices["DISTANCE"].isSubcribed) this._peripheral.devices["DISTANCE"].subscribe();
		switch (args.SENSOR) {
		case 'distance':
			value = this._peripheral.devices["DISTANCE"].distance;
			break;
		case 'color':
			value = this._peripheral.devices["DISTANCE"].color;
		}
        return value;
    }
    
    whenTiltSensor (args) {
		if (!("TILT" in this._peripheral.devices)) return false;
		if  (!this._peripheral.devices["TILT"].isSubcribed) this._peripheral.devices["TILT"].subscribe();
		switch (args.SENSOR) {
		case 'roll': 
			value = this._peripheral.devices["TILT"].roll;
			break;
		case 'pitch': 
			value = this._peripheral.devices["TILT"].pitch;
			break;
		case 'yaw': 
			value = this._peripheral.devices["TILT"].yaw;
			break;
		}
        switch (args.OP) {
        case '<':
            return value < Cast.toNumber(args.REFERENCE);
        case '>':
            return value > Cast.toNumber(args.REFERENCE);
        default:
            console.log(`Unknown comparison operator in whenTilt: ${args.OP}`);
            return false;
        }
    }
    getTiltSensor (args) {
		if (!("TILT" in this._peripheral.devices)) return 0;
		if  (!this._peripheral.devices["TILT"].isSubcribed) this._peripheral.devices["TILT"].subscribe();
		switch (args.SENSOR) {
		case 'roll': 
			value = this._peripheral.devices["TILT"].roll;
			break;
		case 'pitch': 
			value = this._peripheral.devices["TILT"].pitch;
			break;
		case 'yaw':
			value = this._peripheral.devices["TILT"].yaw;
			break;
		}
        return value;
    }
    getElectricalSensor (args) {
		sensor = args.SENSOR.toUpperCase();
		if (!(sensor in this._peripheral.devices)) return 0;
		if  (!this._peripheral.devices[sensor].isSubcribed) this._peripheral.devices[sensor].subscribe();
        return this._peripheral.devices[sensor].value;
    }
}

module.exports = Scratch3BoostBlocks;
