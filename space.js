/// <reference types="@types/lodash" />

/**
 * @typedef {[number, number]} Vector
 */

/**
 * @typedef {object} Ship
 * @prop {number} mass
 * @prop {number} inertia
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

/** @type {Vector} */
const center = [0, 0]

/** @type {Vector[]} */
const shipBody = [
  [0, 0],
  [1, -1],
  [0, 2],
  [-1, -1],
  [0, 0],
]

const shipRadius = Math.max(
  ...shipBody.map((point) => getDistance(point, center))
)

/** @type {Vector[]} */
const shipThrust = [
  [-0.25, -0.25],
  [0, -1],
  [0.25, -0.25],
]

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
function getDistance(a, b) {
  return Math.sqrt(
    Math.pow(getDifferenceX(a, b), 2) + Math.pow(getDifferenceY(a, b), 2)
  )
}

/**
 * @param {Vector} v
 * @returns {number}
 */
function getMagnitude(v) {
  return Math.sqrt(Math.pow(v[0], 2) + Math.pow(v[1], 2))
}

/**
 * @param {Vector} a
 * @param {Vector} b
 * @returns {Vector}
 */
function getDifference(a, b) {
  return [b[0] - a[0], b[1] - a[1]]
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
  const distance = getDistance(a.location, b.location)
  const force = getGravityForce(a.mass, b.mass, distance)
  return [
    (force * getDifferenceX(a.location, b.location)) / distance,
    (force * getDifferenceY(a.location, b.location)) / distance,
  ]
}

function getBodyRadius(body) {
  return Math.pow(((body.mass / Math.PI) * 3) / 2, 1 / 3)
}

function tick(force) {
  STATE.rendered = false

  if (STATE.ship.destroyed && STATE.ship.destroyedFor > 2) {
    reset()
  }

  STATE.time += STATE.timeInc

  STATE.bodies = getSystemBodies(SYSTEM, STATE.time)

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

  if (STATE.ship.destroyed) {
    // Increment destroyed counter
    STATE.ship.destroyedFor += STATE.timeInc
    // Slowly decrease velocity (and player view)
    STATE.ship.velocity = scale(STATE.ship.velocity, 0.99)
  }

  if (!STATE.ship.destroyed) {
    // Apply gravity to ship
    STATE.ship.velocity = translate(
      STATE.ship.velocity,
      scale(
        getForceOn(STATE.ship, STATE.bodies),
        STATE.timeInc / STATE.ship.mass
      )
    )
  }

  // Move ship
  STATE.ship.location = translate(
    STATE.ship.location,
    scale(STATE.ship.velocity, STATE.timeInc)
  )
  STATE.ship.angle += STATE.ship.angularVelocity * STATE.timeInc
  STATE.ship.angle %= Math.PI * 2

  STATE.ship.colliding = false
  STATE.ship.landed = false

  if (!STATE.ship.destroyed) {
    for (const body of STATE.bodies) {
      const distance = getDistance(body.location, STATE.ship.location)
      const radius = getBodyRadius(body)

      if (distance < radius + shipRadius) {
        const points = shipBody.map((v) =>
          translate(rotate(v, center, STATE.ship.angle), STATE.ship.location)
        )
        /** @type {[Vector, number][]} */
        const contactPoints = []
        STATE.debugShapes = []
        for (let index = 0; index < points.length - 1; index++) {
          const a = points[index]
          const aDistance = getDistance(a, body.location)
          if (aDistance < radius) {
            contactPoints.push([a, aDistance])
          }
          const b = points[index + 1]
          const length = getDistance(a, b)
          const dot =
            ((body.location[0] - a[0]) * (b[0] - a[0]) +
              (body.location[1] - a[1]) * (b[1] - a[1])) /
            Math.pow(length, 2)
          const closest = [
            a[0] + dot * (b[0] - a[0]),
            a[1] + dot * (b[1] - a[1]),
          ]
          const closestDistance = getDistance(body.location, closest)
          if (
            closestDistance < radius &&
            getDistance(a, closest) + getDistance(closest, b) < length + 0.01
          ) {
            contactPoints.push([closest, closestDistance])
          }
        }

        // Process the closest points first, they have penetrated deepest.
        contactPoints.sort((a, b) => a[1] - b[1])

        for (const [point, pointBodyDistance] of contactPoints) {
          const shipPoint = getDifference(point, STATE.ship.location)
          const shipPointPerp = perpendicular(shipPoint)
          const shipPointAngularVelocity = scale(
            shipPointPerp,
            STATE.ship.angularVelocity
          )
          const pointRelativeVelocity = getDifference(
            body.velocity,
            translate(STATE.ship.velocity, shipPointAngularVelocity)
          )
          const bodyPoint = getDifference(body.location, point)
          const collisionNormal = normalize(bodyPoint)
          const colliding = dot(collisionNormal, pointRelativeVelocity) < 0
          STATE.ship.colliding = STATE.ship.colliding || colliding
          if (colliding) {
            const impulseMagnitude =
              -dot(scale(pointRelativeVelocity, 1), collisionNormal) /
              (dot(
                collisionNormal,
                scale(collisionNormal, 1 / STATE.ship.mass)
              ) +
                Math.pow(dot(shipPointPerp, collisionNormal), 2) /
                  STATE.ship.inertia)
            if (impulseMagnitude > 4) {
              STATE.ship.destroyed = true
              STATE.ship.destroyedFor = 0
              return
            }
            STATE.ship.velocity = translate(
              STATE.ship.velocity,
              scale(collisionNormal, impulseMagnitude / STATE.ship.mass)
            )
            STATE.ship.angularVelocity +=
              dot(shipPointPerp, scale(collisionNormal, impulseMagnitude)) /
              STATE.ship.inertia
            // Friction
            const collisionPerp = perpendicular(collisionNormal)
            const surfaceVelocity = dot(pointRelativeVelocity, collisionPerp)
            STATE.ship.velocity = translate(
              STATE.ship.velocity,
              scale(collisionPerp, -surfaceVelocity / 20)
            )
          }
          // Shift out of body.
          STATE.ship.location = translate(
            STATE.ship.location,
            scale(collisionNormal, (radius - pointBodyDistance) / 2)
          )
        }

        STATE.ship.landed =
          STATE.ship.colliding &&
          Math.abs(STATE.ship.angularVelocity) < 1 &&
          getDistance(STATE.ship.velocity, body.velocity) < 0.5 &&
          Math.abs(
            getAngleDifference(
              STATE.ship.angle,
              getAngle(STATE.ship.location, body.location)
            )
          ) < 0.5
      }
    }
  }
}

/**
 * @param {Shape[]} shapes
 */
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

/**
 * @param {Vector} xy
 * @param {Vector} by
 * @returns {Vector}
 */
function translate(xy, by) {
  return [xy[0] + by[0], xy[1] + by[1]]
}

/**
 * @param {Vector} v
 * @returns {Vector}
 */
function normalize(v) {
  const size = getMagnitude(v)
  if (size === 0) return v
  return [v[0] / size, v[1] / size]
}

/**
 * @param {Vector} a
 * @param {Vector} b
 * @returns {number}
 */
function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1]
}

/**
 * @param {Shape[]} shapes
 * @param {Vector} by
 * @returns {Shape[]}
 */
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

/**
 * @param {Vector} xy
 * @param {number} factor
 * @returns {Vector}
 */
function scale(xy, factor) {
  return [xy[0] * factor, xy[1] * factor]
}

/**
 * @param {Vector} vector
 * @returns {Vector}
 */
function perpendicular(vector) {
  return [-vector[1], vector[0]]
}

/**
 * @param {Shape[]} shapes
 * @param {number} factor
 * @returns {Shape[]}
 */
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

/**
 * @param {Vector} xy
 * @param {Vector} about
 * @param {number} angle
 * @returns {Vector}
 */
function rotate(xy, about, angle) {
  const distance = getDistance(xy, about)
  const currentAngle = getAngle(xy, about)
  return [
    about[0] + Math.sin(currentAngle + angle) * distance,
    about[1] + Math.cos(currentAngle + angle) * distance,
  ]
}

/**
 * @param {Shape[]} shapes
 * @param {Vector} about
 * @param {number} angle
 * @returns {Shape[]}
 */
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

/**
 * @typedef {object} Circle
 * @prop {'circle'} type
 * @prop {Vector} center
 * @prop {number} radius
 * @prop {string} strokeStyle
 */

/**
 * @typedef {object} Line
 * @prop {'line'} type
 * @prop {Vector[]} points
 * @prop {string} strokeStyle
 */

/**
 * @typedef {Circle | Line} Shape
 */

/**
 * @param {Ship} ship
 */
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
        points: shipThrust,
        strokeStyle: "#ff0",
      })
    }
    // Ship body
    shapes.push({
      type: "line",
      points: shipBody,
      strokeStyle: ship.landed ? "#3f3" : ship.colliding ? "#f33" : "#33f",
    })
  }

  return translateShapes(
    rotateShapes(shapes, [0, 0], ship.angle),
    ship.location
  )
}

function renderWorld() {
  const shapes = []

  STATE.debugShapes?.forEach((shape) => {
    shapes.push(shape)
  })

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
  if (!STATE.rendered) {
    clearCanvas()
    renderWorld()
    STATE.rendered = true
  }
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
  if (e.key === "r") {
    reset()
    tick(true)
  }
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
  STATE.rendered = false
  STATE.time = 0
  STATE.timeInc = 1 / 100
  STATE.zoom = 10
  STATE.bodies = getSystemBodies(SYSTEM, STATE.time)
  const bodyToStartOn = STATE.bodies[_.random(STATE.bodies.length - 2) + 1]
  STATE.ship = {
    mass: 1,
    inertia: 1,
    location: translate(bodyToStartOn.location, [
      0,
      -getBodyRadius(bodyToStartOn) - shipRadius + 0.5,
    ]),
    velocity: bodyToStartOn.velocity,
    angle: Math.PI,
    angularVelocity: 0,
    thrusting: false,
    landed: false,
  }
}

reset()

setInterval(tick, 15)
requestAnimationFrame(render)
