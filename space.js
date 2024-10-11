/// <reference types="@types/lodash" />

const l = console.log.bind(console)
const shipRadius = 1

/**
 * @typedef {[number, number]} Vector
 */

/**
 * @typedef {object} Ship
 * @prop {number} mass
 * @prop {Vector} location
 * @prop {Vector} velocity
 * @prop {number} angle
 * @prop {number} angularVelocity
 * @prop {boolean} thrusting
 * @prop {boolean} landed
 * @prop {boolean} takingOff
 * @prop {boolean} destroyed
 * @prop {number} destroyedFor
 */

/**
 * @typedef {object} System
 * @prop {number} mass
 * @prop {number} orbitRadius
 * @prop {System[]} [children]
 */

/**
 * @typedef {object} SystemBody
 * @prop {number} mass
 * @prop {Vector} location
 * @prop {Vector} velocity
 */

/**
 * @typedef {object} State
 * @prop {number}
 * @prop {number} time
 * @prop {number} timeInc
 * @prop {number} zoom
 * @prop {SystemBody[]} bodies
 * @prop {Ship} ship
 */

/** @type {State} */
const STATE = {}

function getTotalMass(bodies) {
  return bodies.reduce((memo, body) => {
    return memo + body.mass
  }, 0)
}

function getOrbitVelocity(parentMass, childMass, radius) {
  const force = getGravityForce(parentMass, childMass, radius)
  return Math.sqrt((force / childMass) * radius)
}

/**
 * @param {System} system
 * @param {number} time
 * @returns {SystemBody[]}
 */
function getSystemBodies(system, time) {
  const bodies = [
    {
      mass: system.mass,
      location: [0, 0],
      velocity: [0, 0],
    },
  ]
  if (system.children) {
    system.children.forEach((child) => {
      // Calculate child location
      const childBodies = getSystemBodies(child, time)
      if (!child.orbitVelocity) {
        child.orbitVelocity = getOrbitVelocity(
          system.mass,
          getTotalMass(childBodies),
          child.orbitRadius
        )
      }
      // velocity = distance / time
      // diameter = 2 * PI * radiusPx
      // rotation/time = velocity / diameter
      // angle/time = velocity / radius
      const childAngle = (child.orbitVelocity / child.orbitRadius) * time
      const childLocation = [
        Math.cos(childAngle) * child.orbitRadius,
        Math.sin(childAngle) * child.orbitRadius,
      ]
      const childVelocity = [
        Math.sin(childAngle) * -child.orbitVelocity,
        Math.cos(childAngle) * child.orbitVelocity,
      ]
      childBodies.forEach((body) => {
        body.location = translate(body.location, childLocation)
        body.velocity = translate(body.velocity, childVelocity)
        if (body.orbitCenter) {
          body.orbitCenter = translate(body.orbitCenter, childLocation)
        } else {
          body.orbitCenter = [0, 0]
        }
        if (!body.orbitRadius) {
          body.orbitRadius = child.orbitRadius
        }
        bodies.push(body)
      })
    })
  }
  return bodies
}

function clearCanvas() {
  const width = window.innerWidth * devicePixelRatio
  const height = window.innerHeight * devicePixelRatio
  canvas.width = width
  canvas.height = height
  ctx.fillRect(0, 0, width, height)
}

function drawCircle(x, y, r, style) {
  ctx.beginPath()
  ctx.arc(x, y, r, 0, Math.PI * 2, false)
  ctx.strokeStyle = style
  ctx.stroke()
}

function drawStar(x, y, r, points, depth, style) {
  const lines = points * 2
  ctx.beginPath()
  ctx.moveTo(x, y + r)
  for (let i = 0; i <= lines; i++) {
    ctx.lineTo(
      x + Math.sin((Math.PI * 2 * i) / lines) * r * (i % 2 ? depth : 1),
      y + Math.cos((Math.PI * 2 * i) / lines) * r * (i % 2 ? depth : 1)
    )
  }
  ctx.strokeStyle = style
  ctx.stroke()
}

/**
 * @param {SystemBody} body
 * @param {SystemBody[]} bodies
 * @returns {Vector}
 */
function getForceOn(body, bodies) {
  return bodies.reduce(
    (memo, otherBody) => {
      if (body === otherBody) {
        return memo
      }
      const force = getForceBetween(body, otherBody)
      return [memo[0] + force[0], memo[1] + force[1]]
    },
    [0, 0]
  )
}

/**
 * @param {Vector} a
 * @param {Vector} b
 * @returns {number}
 */
function getDifference(a, b) {
  return Math.sqrt(
    Math.pow(getDifferenceX(a, b), 2) + Math.pow(getDifferenceY(a, b), 2)
  )
}

/**
 * @param {Vector} a
 * @param {Vector} b
 * @returns {number}
 */
function getDifferenceX(a, b) {
  return b[0] - a[0]
}

/**
 * @param {Vector} a
 * @param {Vector} b
 * @returns {number}
 */
function getDifferenceY(a, b) {
  return b[1] - a[1]
}

/**
 * @param {Vector} a
 * @param {Vector} b
 * @returns {number}
 */
function getAngle(a, b) {
  return Math.atan2(a[0] - b[0], a[1] - b[1])
}

/**
 * @param {number} angle
 * @param {number} distance
 * @returns {Vector}
 */
function getAngleVector(angle, distance) {
  return [Math.sin(angle) * distance, Math.cos(angle) * distance]
}

/**
 * @param {number} a
 * @param {number} b
 * @returns {number}
 */
function getAngleDifference(a, b) {
  const diff = (b - a) % (Math.PI * 2)
  if (diff > Math.PI) {
    return diff - Math.PI * 2
  } else if (diff < -Math.PI) {
    return diff + Math.PI * 2
  }
  return diff
}

/**
 * @param {number} massA
 * @param {number} massB
 * @returns {number}
 */
function getGravityForce(massA, massB, distance) {
  return (massA * massB) / Math.pow(distance, 2)
}

/**
 * @param {SystemBody} a
 * @param {SystemBody} b
 * @returns {Vector}
 */
function getForceBetween(a, b) {
  const distance = getDifference(a.location, b.location)
  const force = getGravityForce(a.mass, b.mass, distance)
  return [
    (force * getDifferenceX(a.location, b.location)) / distance,
    (force * getDifferenceY(a.location, b.location)) / distance,
  ]
}

function getBodyRadius(body) {
  return Math.pow(((body.mass / Math.PI) * 3) / 2, 1 / 3)
}

function tick() {
  if (STATE.ship.destroyed && STATE.ship.destroyedFor > 4) {
    reset()
  }

  STATE.time += STATE.timeInc

  STATE.bodies = getSystemBodies(SYSTEM, STATE.time)

  /** @type {SystemBody} */
  const nearestBody = _.minBy(STATE.bodies, (body) => {
    return getDifference(body.location, STATE.ship.location)
  })

  if (STATE.ship.destroyed) {
    // Increment destroyed counter
    STATE.ship.destroyedFor += STATE.timeInc
    // Slowly decrease velocity (and player view)
    STATE.ship.velocity = scale(STATE.ship.velocity, 0.99)
  } else if (!STATE.ship.landed) {
    // Apply gravity to ship
    STATE.ship.velocity = translate(
      STATE.ship.velocity,
      scale(
        getForceOn(STATE.ship, STATE.bodies),
        STATE.timeInc / STATE.ship.mass
      )
    )

    // Check for collision
    if (
      getBodyRadius(nearestBody) >
      getDifference(nearestBody.location, STATE.ship.location) - shipRadius
    ) {
      const velocity = getDifference(nearestBody.velocity, STATE.ship.velocity)
      const approachAngle = getAngle(STATE.ship.location, nearestBody.location)
      const angleDiff = Math.abs(
        getAngleDifference(STATE.ship.angle, approachAngle)
      )
      if (
        velocity < 10 &&
        angleDiff < 0.5 &&
        Math.abs(STATE.ship.angularVelocity) < 2 &&
        !STATE.ship.thrusting
      ) {
        STATE.ship.landed = true
        STATE.ship.angle = approachAngle
        STATE.ship.angularVelocity = 0
      } else {
        STATE.ship.destroyed = true
        STATE.ship.destroyedFor = 0
      }
    }
  }

  // Handle keys
  if (KEYS["="]) {
    // Zoom in
    STATE.zoom = STATE.zoom * 1.05
  }
  if (KEYS["-"]) {
    // Zoom out
    STATE.zoom = STATE.zoom / 1.05
  }
  if (KEYS["ArrowUp"] && !STATE.ship.destroyed) {
    // Thrust
    STATE.ship.thrusting = true
    STATE.ship.landed = false
    STATE.ship.velocity = translate(
      STATE.ship.velocity,
      getAngleVector(STATE.ship.angle, 0.1)
    )
  } else {
    STATE.ship.thrusting = false
  }
  if (KEYS["ArrowLeft"] && !STATE.ship.landed) {
    // Left
    STATE.ship.angularVelocity += 0.1
  }
  if (KEYS["ArrowRight"] && !STATE.ship.landed) {
    // Right
    STATE.ship.angularVelocity -= 0.1
  }

  // Move ship
  if (STATE.ship.landed) {
    STATE.ship.location = translate(
      nearestBody.location,
      getAngleVector(STATE.ship.angle, getBodyRadius(nearestBody) + shipRadius)
    )
    STATE.ship.velocity = nearestBody.velocity
  } else {
    STATE.ship.location = translate(
      STATE.ship.location,
      scale(STATE.ship.velocity, STATE.timeInc)
    )
    STATE.ship.angle += STATE.ship.angularVelocity * STATE.timeInc
    STATE.ship.angle %= Math.PI * 2
  }
}

function draw(shapes) {
  shapes.forEach((shape) => {
    if (shape.type == "circle") {
      ctx.beginPath()
      ctx.arc(
        shape.center[0] * devicePixelRatio,
        shape.center[1] * devicePixelRatio,
        shape.radius * devicePixelRatio,
        0,
        Math.PI * 2 * devicePixelRatio,
        shape.filled
      )
      ctx.strokeStyle = shape.strokeStyle
      ctx.stroke()
    }
    if (shape.type == "line") {
      ctx.beginPath()
      for (let i = 0; i < shape.points.length; i++) {
        ctx.lineTo(
          shape.points[i][0] * devicePixelRatio,
          shape.points[i][1] * devicePixelRatio
        )
      }
      ctx.strokeStyle = shape.strokeStyle
      ctx.stroke()
    }
  })
}

function translate(xy, by) {
  return [xy[0] + by[0], xy[1] + by[1]]
}

function translateShapes(shapes, by) {
  return shapes.map((shape) => {
    if (shape.type == "circle") {
      shape.center = translate(shape.center, by)
    }
    if (shape.type == "line") {
      shape.points = shape.points.map((point) => {
        return translate(point, by)
      })
    }
    return shape
  })
}

function scale(xy, factor) {
  return [xy[0] * factor, xy[1] * factor]
}

function scaleShapes(shapes, factor) {
  return shapes.map((shape) => {
    if (shape.type == "circle") {
      shape.center = scale(shape.center, factor)
      shape.radius = shape.radius * factor
    }
    if (shape.type == "line") {
      shape.points = shape.points.map((point) => {
        return scale(point, factor)
      })
    }
    return shape
  })
}

function rotate(xy, about, angle) {
  const distance = getDifference(xy, about)
  const currentAngle = getAngle(xy, about)
  return [
    about[0] + Math.sin(currentAngle + angle) * distance,
    about[1] + Math.cos(currentAngle + angle) * distance,
  ]
}

function rotateShapes(shapes, about, angle) {
  return shapes.map((shape) => {
    if (shape.type == "circle") {
      shape.center = rotate(shape.center, about, angle)
    }
    if (shape.type == "line") {
      shape.points = shape.points.map((point) => {
        return rotate(point, about, angle)
      })
    }
    return shape
  })
}

function drawShip(ship) {
  const shapes = []

  if (ship.destroyed) {
    // Circle that gets bigger and transitions from yellow to red/transparent
    shapes.push({
      type: "circle",
      center: [0, 0],
      radius: 1 + 10 * ship.destroyedFor,
      strokeStyle:
        "rgba(255, " +
        Math.round(Math.max(0, 255 - ship.destroyedFor * 1000)) +
        ", 0, " +
        Math.max(0, 1 - ship.destroyedFor * 4) +
        ")",
    })
  } else {
    if (ship.thrusting) {
      // Yellow flame
      shapes.push({
        type: "line",
        points: [
          [-0.25, 0.25],
          [0, 1],
          [0.25, 0.25],
        ],
        strokeStyle: "#ff0",
      })
    }
    // Ship body
    shapes.push({
      type: "line",
      points: [
        [0, 0],
        [1, 1],
        [0, -2],
        [-1, 1],
        [0, 0],
      ],
      strokeStyle: "#33f",
    })
  }

  return translateShapes(
    scaleShapes(rotateShapes(shapes, [0, 0], Math.PI + ship.angle), 1),
    ship.location
  )
}

function renderWorld() {
  const shapes = []

  // Orbit rings
  STATE.bodies.forEach((body) => {
    if (body.orbitCenter) {
      shapes.push({
        type: "circle",
        center: body.orbitCenter,
        radius: body.orbitRadius,
        strokeStyle: "#333",
      })
    }
  })

  // Bodies
  STATE.bodies.forEach((body) => {
    shapes.push({
      type: "circle",
      center: body.location,
      radius: getBodyRadius(body),
      strokeStyle: "#fff",
    })
  })

  // Ship
  shapes.push(...drawShip(STATE.ship))

  draw(
    translateShapes(
      scaleShapes(
        translateShapes(shapes, [
          -STATE.ship.location[0],
          -STATE.ship.location[1],
        ]),
        STATE.zoom
      ),
      [canvas.clientWidth / 2, canvas.clientHeight / 2]
    )
  )
}

function render() {
  clearCanvas()
  renderWorld()
  requestAnimationFrame(render)
}

// Prepare canvas
const canvas = document.createElement("canvas")
document.body.appendChild(canvas)
const ctx = canvas.getContext("2d")

// Set event listeners
const KEYS = {}
document.addEventListener("keydown", (e) => {
  KEYS[e.key] = true
})
document.addEventListener("keyup", (e) => {
  KEYS[e.key] = false
})

/** @type {System} */
const SYSTEM = {
  mass: 10000,
  orbitRadius: 0,
  children: [
    {
      mass: 20,
      orbitRadius: 50,
    },
    {
      mass: 30,
      orbitRadius: 100,
    },
    {
      mass: 50,
      orbitRadius: 200,
      children: [
        {
          mass: 20,
          orbitRadius: 10,
        },
      ],
    },
    {
      mass: 200,
      orbitRadius: 400,
      children: [
        {
          mass: 20,
          orbitRadius: 20,
        },
        {
          mass: 10,
          orbitRadius: 50,
        },
      ],
    },
  ],
}

function reset() {
  STATE.time = 0
  STATE.timeInc = 1 / 100
  STATE.zoom = 10
  STATE.bodies = getSystemBodies(SYSTEM, STATE.time)
  const bodyToStartOn = STATE.bodies[_.random(STATE.bodies.length - 2) + 1]
  STATE.ship = {
    mass: 1,
    location: translate(bodyToStartOn.location, [
      0,
      -getBodyRadius(bodyToStartOn) - shipRadius,
    ]),
    velocity: bodyToStartOn.velocity,
    angle: Math.PI,
    angularVelocity: 0,
    thrusting: false,
    landed: true,
  }
}

reset()

setInterval(tick, 15)
requestAnimationFrame(render)
