
"use strict";

let AnalogWrite = require('./AnalogWrite.js')
let ServoWrite = require('./ServoWrite.js')
let AnalogRead = require('./AnalogRead.js')
let DigitalWrite = require('./DigitalWrite.js')
let DigitalSetDirection = require('./DigitalSetDirection.js')
let DigitalRead = require('./DigitalRead.js')
let ServoRead = require('./ServoRead.js')

module.exports = {
  AnalogWrite: AnalogWrite,
  ServoWrite: ServoWrite,
  AnalogRead: AnalogRead,
  DigitalWrite: DigitalWrite,
  DigitalSetDirection: DigitalSetDirection,
  DigitalRead: DigitalRead,
  ServoRead: ServoRead,
};
