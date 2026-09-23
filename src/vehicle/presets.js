// Vehicle presets. Global coordinates: x forward (front axle at x = 0, rear axle at
// x = -wheelbase), y left, z up from the ground, millimetres.
// Suspension hardpoints are LEFT-side, axle-local x. Default hardpoints were tuned with
// the built-in suspension optimisers (roll centre / anti / bump steer / Ackermann / MR).
// Direct-acting coilover corners carry unused rocker points.

const corner = (o) => ({
  camber: -1, toe: 0, tyreR: 330, tyreW: 245, actuation: 'direct', pushOn: 'lca',
  rackMax: 0, springRate: 60, arbRate: 0, ...o,
});

export const PRESETS = {
  supercar: () => ({
    type: 'supercar',
    name: 'Mid-engine supercar - bonded aluminium tub',
    vehicle: { wheelbase: 2650, brakeFront: 0.62, driveFront: 0, aeroNote: '' },
    chassis: {
      xNose: 700, xFront: -250, xDash: -700, xSeat: -1900, xRear: -3350,
      floorZ: 110, W: 720, sillW: 200, sillH: 320,
      tunnel: true, tunnelHalf: 120, tunnelH: 240,
      deck: true, deckH: 480, firewallH: 620,
      taperW: 1.0, taperH: 1.0,
      battery: false, batteryH: 140, batteryBays: 5,
      frontRails: true, frRailY: [300, 440], frRailZ: [130, 690],
      rearRails: true, rrRailY: [380, 500], rrRailZ: [130, 710],
      rollHoop: true, hoopH: 1060, cage: true, roofH: 1060, frontHoop: false,
      rearBrace: true, engine: false, engineEnd: -2700, bumpers: true,
      hpSpread: 110, mesh: 75,
    },
    material: 'al5754',
    gauges: { floor: 3.0, sills: 3.0, tunnel: 2.5, bulkheads: 2.5, deck: 2.0, frontRails: 3.0, rearRails: 3.0, battery: 2.0 },
    tubes: { hoop: { od: 45, wall: 2.5, mat: 's355' }, cage: { od: 38, wall: 2.0, mat: 's355' }, brace: { od: 35, wall: 2.0, mat: 's355' }, engine: { od: 80, wall: 6, mat: 'al6082' } },
    suspension: {
      front: corner({
        camber: -1.2, toe: 0.05, tyreR: 330, tyreW: 245, rackMax: 70, cFactor: 55, springRate: 28.5, arbRate: 106,
        hp: {
          wc: [0, 830, 330], lcaO: [0, 745, 155], lcaF: [230, 284, 113], lcaR: [-230, 353, 164],
          ucaO: [-30, 705, 520], ucaF: [150, 470, 416], ucaR: [-170, 509, 500], tieO: [-131, 720, 183],
          tieI: [-140, 310, 161], pushO: [-20, 667, 175], rockP: [-40, 410, 680], rockA: [60, 410, 680],
          rockPush: [-40, 470, 680], rockDamp: [-40, 470, 680], damperC: [-40, 493, 680],
        },
      }),
      rear: corner({
        camber: -1.8, toe: 0.15, tyreR: 350, tyreW: 305, springRate: 68.5, arbRate: 68,
        hp: {
          wc: [0, 810, 350], lcaO: [0, 735, 150], lcaF: [240, 353, 207], lcaR: [-200, 366, 89],
          ucaO: [10, 695, 540], ucaF: [180, 548, 456], ucaR: [-160, 538, 529], tieO: [-150, 715, 198],
          tieI: [-150, 320, 177], pushO: [20, 666, 175], rockP: [40, 420, 700], rockA: [140, 420, 700],
          rockPush: [40, 480, 700], rockDamp: [40, 480, 700], damperC: [40, 492, 700],
        },
      }),
    },
    masses: [
      { name: 'Driver', m: 80, x: -1500, y: 250, z: 400 },
      { name: 'Passenger', m: 0, x: -1500, y: -250, z: 400 },
      { name: 'Engine + gearbox', m: 235, x: -2450, y: 0, z: 400 },
      { name: 'Fuel (half tank)', m: 35, x: -1980, y: 0, z: 420 },
      { name: 'Radiators + front ancillaries', m: 38, x: 520, y: 0, z: 380 },
      { name: 'Body panels + glazing', m: 115, x: -1450, y: 0, z: 720 },
      { name: 'Interior + electrics', m: 75, x: -1250, y: 0, z: 450 },
      { name: 'Exhaust', m: 22, x: -3250, y: 0, z: 450 },
      { name: 'Ballast', m: 0, x: -300, y: 0, z: 160 },
    ],
    unsprung: { front: 42, rear: 48 },
    ride: { fF: 1.7, fR: 1.9, rollGrad: 0.75, lltd: 48, tyreRateF: 280, tyreRateR: 320, dampRatio: 0.35 },
    analysis: { targetK: 35000 },
  }),

  single: () => ({
    type: 'single',
    name: 'Single-seater - sheet aluminium monocoque, stressed engine',
    vehicle: { wheelbase: 2750, brakeFront: 0.58, driveFront: 0 },
    chassis: {
      xNose: 500, xFront: 500, xDash: -350, xSeat: -1350, xRear: -3050,
      floorZ: 40, W: 380, sillW: 70, sillH: 480,
      tunnel: false, tunnelHalf: 100, tunnelH: 200,
      deck: true, deckH: 520, firewallH: 650,
      taperW: 0.5, taperH: 0.72,
      battery: false, batteryH: 120, batteryBays: 4,
      frontRails: false, frRailY: [200, 300], frRailZ: [60, 400],
      rearRails: false, rrRailY: [200, 300], rrRailZ: [60, 400],
      rollHoop: true, hoopH: 1000, cage: false, roofH: 1000, frontHoop: true,
      rearBrace: false, engine: true, engineEnd: -2150, bumpers: false,
      hpSpread: 110, mesh: 60,
    },
    material: 'al5754',
    gauges: { floor: 2.5, sills: 2.0, tunnel: 1.6, bulkheads: 2.5, deck: 2.0, frontRails: 2.0, rearRails: 2.0, battery: 1.6 },
    tubes: { hoop: { od: 40, wall: 2.5, mat: 'crmo' }, cage: { od: 30, wall: 1.6, mat: 'crmo' }, brace: { od: 30, wall: 1.6, mat: 'crmo' }, engine: { od: 70, wall: 6, mat: 'al6082' } },
    suspension: {
      front: corner({
        camber: -2.0, toe: -0.1, tyreR: 280, tyreW: 230, actuation: 'pushrod', pushOn: 'lca', rackMax: 30, cFactor: 60, springRate: 36.5, arbRate: 231,
        hp: {
          wc: [0, 740, 280], lcaO: [5, 680, 135], lcaF: [250, 175, 111], lcaR: [-230, 185, 115],
          ucaO: [-25, 655, 400], ucaF: [230, 275, 300], ucaR: [-220, 285, 295], tieO: [115, 640, 390],
          tieI: [25, 268, 289], pushO: [0, 615, 170], rockP: [-30, 150, 470], rockA: [70, 150, 470],
          rockPush: [-30, 166, 495], rockDamp: [-30, 90, 470], damperC: [-30, 85, 180],
        },
      }),
      rear: corner({
        camber: -1.5, toe: 0.2, tyreR: 300, tyreW: 300, actuation: 'pushrod', pushOn: 'lca', springRate: 75.5, arbRate: 159,
        hp: {
          wc: [0, 710, 300], lcaO: [0, 650, 130], lcaF: [250, 135, 124], lcaR: [-150, 115, 118],
          ucaO: [0, 620, 420], ucaF: [220, 225, 335], ucaR: [-170, 210, 321], tieO: [-120, 640, 171],
          tieI: [-120, 142, 147], pushO: [0, 590, 150], rockP: [-40, 80, 470], rockA: [60, 80, 470],
          rockPush: [-40, 96, 495], rockDamp: [-40, 130, 470], damperC: [-40, 135, 740],
        },
      }),
    },
    masses: [
      { name: 'Driver', m: 75, x: -950, y: 0, z: 300 },
      { name: 'Engine', m: 110, x: -1750, y: 0, z: 300 },
      { name: 'Gearbox + diff', m: 42, x: -2650, y: 0, z: 280 },
      { name: 'Fuel', m: 35, x: -1300, y: 0, z: 320 },
      { name: 'Bodywork + wings', m: 55, x: -1300, y: 0, z: 550 },
      { name: 'Radiators', m: 18, x: -1400, y: 0, z: 350 },
      { name: 'Electrics + ECU', m: 15, x: -700, y: 0, z: 400 },
      { name: 'Ballast', m: 10, x: -600, y: 0, z: 60 },
    ],
    unsprung: { front: 18, rear: 22 },
    ride: { fF: 3.2, fR: 3.5, rollGrad: 0.25, lltd: 50, tyreRateF: 220, tyreRateR: 260, dampRatio: 0.5 },
    analysis: { targetK: 12000 },
  }),

  ev: () => ({
    type: 'ev',
    name: 'EV coupe - structural battery skateboard',
    vehicle: { wheelbase: 2900, brakeFront: 0.65, driveFront: 0.4 },
    chassis: {
      xNose: 850, xFront: -300, xDash: -700, xSeat: -2450, xRear: -3800,
      floorZ: 130, W: 780, sillW: 170, sillH: 190,
      tunnel: false, tunnelHalf: 100, tunnelH: 200,
      deck: true, deckH: 600, firewallH: 500,
      taperW: 1.0, taperH: 1.0,
      battery: true, batteryH: 140, batteryBays: 5,
      frontRails: true, frRailY: [320, 460], frRailZ: [150, 760],
      rearRails: true, rrRailY: [360, 500], rrRailZ: [150, 760],
      rollHoop: true, hoopH: 1100, cage: true, roofH: 1100, frontHoop: false,
      rearBrace: true, engine: false, engineEnd: -3000, bumpers: true,
      hpSpread: 110, mesh: 90,
    },
    material: 'al6082',
    gauges: { floor: 3.0, sills: 3.5, tunnel: 2.0, bulkheads: 2.5, deck: 2.0, frontRails: 3.0, rearRails: 3.0, battery: 2.5 },
    tubes: { hoop: { od: 50, wall: 3, mat: 'al6082' }, cage: { od: 45, wall: 3, mat: 'al6082' }, brace: { od: 40, wall: 2.5, mat: 'al6082' }, engine: { od: 80, wall: 6, mat: 'al6082' } },
    suspension: {
      front: corner({
        camber: -0.8, toe: 0.05, tyreR: 340, tyreW: 255, rackMax: 75, cFactor: 50, springRate: 58, arbRate: 153,
        hp: {
          wc: [0, 820, 340], lcaO: [0, 740, 160], lcaF: [220, 360, 153], lcaR: [-260, 360, 174],
          ucaO: [-35, 700, 590], ucaF: [120, 509, 531], ucaR: [-180, 510, 545], tieO: [-154, 717, 160],
          tieI: [-180, 346, 164], pushO: [-20, 641, 180], rockP: [-40, 460, 760], rockA: [60, 460, 760],
          rockPush: [-40, 520, 760], rockDamp: [-40, 520, 760], damperC: [-40, 540, 760],
        },
      }),
      rear: corner({
        camber: -1.2, toe: 0.2, tyreR: 345, tyreW: 285, springRate: 69.5, arbRate: 71.5,
        hp: {
          wc: [0, 820, 345], lcaO: [0, 745, 150], lcaF: [240, 400, 169], lcaR: [-220, 400, 138],
          ucaO: [10, 700, 580], ucaF: [170, 540, 520], ucaR: [-170, 540, 525], tieO: [-150, 720, 170],
          tieI: [-150, 390, 167], pushO: [20, 658, 180], rockP: [40, 460, 750], rockA: [140, 460, 750],
          rockPush: [40, 520, 750], rockDamp: [40, 520, 750], damperC: [40, 532, 750],
        },
      }),
    },
    masses: [
      { name: 'Battery modules', m: 470, x: -1380, y: 0, z: 200 },
      { name: 'Front drive unit', m: 70, x: 0, y: 0, z: 330 },
      { name: 'Rear drive unit', m: 95, x: -2900, y: 0, z: 330 },
      { name: 'Driver', m: 80, x: -1600, y: 380, z: 480 },
      { name: 'Passenger', m: 80, x: -1600, y: -380, z: 480 },
      { name: 'Body closures + glazing', m: 240, x: -1500, y: 0, z: 800 },
      { name: 'Interior + seats', m: 140, x: -1650, y: 0, z: 520 },
      { name: 'HVAC + power electronics', m: 65, x: 200, y: 0, z: 520 },
      { name: 'Ballast', m: 0, x: -1400, y: 0, z: 200 },
    ],
    unsprung: { front: 50, rear: 55 },
    ride: { fF: 1.35, fR: 1.5, rollGrad: 1.0, lltd: 55, tyreRateF: 300, tyreRateR: 320, dampRatio: 0.3 },
    analysis: { targetK: 30000 },
  }),
};

export const PRESET_LIST = [
  ['supercar', 'Supercar (mid-engine)'],
  ['single', 'Single seater'],
  ['ev', 'EV skateboard'],
];

export function makePreset(key) {
  return PRESETS[key]();
}
