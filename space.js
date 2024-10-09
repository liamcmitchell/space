/// <reference types="@types/lodash" />

var l = console.log.bind(console)

var STATE = {}

function getTotalMass(bodies) {
  return bodies.reduce(function (memo, body) {
    return memo + body.mass
  }, 0)
}

function getOrbitVelocity(parentMass, childMass, radius) {
  var force = getGravityForce(parentMass, childMass, radius)
  return Math.sqrt((force / childMass) * radius)
}

function getSystemBodies(system, time) {
  var bodies = [
    {
      mass: system.mass,
      location: [0, 0],
      velocity: [0, 0],
    },
  ]
  if (system.children) {
    system.children.forEach(function (child) {
      // Calculate child location
      var childBodies = getSystemBodies(child, time)
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
      var childAngle = (child.orbitVelocity / child.orbitRadius) * time
      var childLocation = [
        Math.cos(childAngle) * child.orbitRadius,
        Math.sin(childAngle) * child.orbitRadius,
      ]
      var childVelocity = [
        Math.sin(childAngle) * -child.orbitVelocity,
        Math.cos(childAngle) * child.orbitVelocity,
      ]
      childBodies.forEach(function (body) {
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
  ctx.fillRect(0, 0, canvas.clientWidth, canvas.clientHeight)
}

function drawCircle(x, y, r, style) {
  ctx.beginPath()
  ctx.arc(x, y, r, 0, Math.PI * 2, false)
  ctx.strokeStyle = style
  ctx.stroke()
}

function drawStar(x, y, r, points, depth, style) {
  var lines = points * 2
  ctx.beginPath()
  ctx.moveTo(x, y + r)
  for (var i = 0; i <= lines; i++) {
    ctx.lineTo(
      x + Math.sin((Math.PI * 2 * i) / lines) * r * (i % 2 ? depth : 1),
      y + Math.cos((Math.PI * 2 * i) / lines) * r * (i % 2 ? depth : 1)
    )
  }
  ctx.strokeStyle = style
  ctx.stroke()
}

function getForceOn(body, bodies) {
  return bodies.reduce(
    function (memo, otherBody) {
      if (body === otherBody) {
        return memo
      }
      var force = getForceBetween(body, otherBody)
      return [memo[0] + force[0], memo[1] + force[1]]
    },
    [0, 0]
  )
}

function getDistance(a, b) {
  return Math.sqrt(
    Math.pow(getDistanceX(a, b), 2) + Math.pow(getDistanceY(a, b), 2)
  )
}

function getDistanceX(a, b) {
  return b[0] - a[0]
}

function getDistanceY(a, b) {
  return b[1] - a[1]
}

function getAngle(a, b) {
  return Math.atan2(a[0] - b[0], a[1] - b[1])
}

function getGravityForce(massA, massB, distance) {
  return (massA * massB) / Math.pow(distance, 2)
}

function getForceBetween(a, b) {
  var distance = getDistance(a.location, b.location)
  var force = getGravityForce(a.mass, b.mass, distance)
  return [
    (force * getDistanceX(a.location, b.location)) / distance,
    (force * getDistanceY(a.location, b.location)) / distance,
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
    STATE.ship.velocity = translate(STATE.ship.velocity, [
      Math.sin(STATE.ship.angle) * 0.1,
      Math.cos(STATE.ship.angle) * 0.1,
    ])
  } else {
    STATE.ship.thrusting = false
  }
  if (KEYS["ArrowLeft"]) {
    // Left
    STATE.ship.angle += 0.1
  }
  if (KEYS["ArrowRight"]) {
    // Right
    STATE.ship.angle -= 0.1
  }

  if (STATE.ship.destroyed) {
    // Increment destroyed counter
    STATE.ship.destroyedFor += STATE.timeInc
    // Slowly decrease velocity (and player view)
    STATE.ship.velocity = scale(STATE.ship.velocity, 0.99)
  } else {
    // Apply gravity to ship
    STATE.ship.velocity = translate(
      STATE.ship.velocity,
      scale(
        getForceOn(STATE.ship, STATE.bodies),
        STATE.timeInc / STATE.ship.mass
      )
    )

    // Check for collision
    var nearestBody = _.minBy(STATE.bodies, function (body) {
      return getDistance(body.location, STATE.ship.location)
    })
    if (
      getBodyRadius(nearestBody) >
      getDistance(nearestBody.location, STATE.ship.location) - 1
    ) {
      STATE.ship.destroyed = true
      STATE.ship.destroyedFor = 0
    }

    // How to land...? I want actual physics...
  }

  // Move ship
  STATE.ship.location = translate(
    STATE.ship.location,
    scale(STATE.ship.velocity, STATE.timeInc)
  )
}

function draw(shapes) {
  shapes.forEach(function (shape) {
    if (shape.type == "circle") {
      ctx.beginPath()
      ctx.arc(
        shape.center[0],
        shape.center[1],
        shape.radius,
        0,
        Math.PI * 2,
        shape.filled
      )
      ctx.strokeStyle = shape.strokeStyle
      ctx.stroke()
    }
    if (shape.type == "line") {
      ctx.beginPath()
      for (var i = 0; i < shape.points.length; i++) {
        ctx.lineTo(shape.points[i][0], shape.points[i][1])
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
  return shapes.map(function (shape) {
    if (shape.type == "circle") {
      shape.center = translate(shape.center, by)
    }
    if (shape.type == "line") {
      shape.points = shape.points.map(function (point) {
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
  return shapes.map(function (shape) {
    if (shape.type == "circle") {
      shape.center = scale(shape.center, factor)
      shape.radius = shape.radius * factor
    }
    if (shape.type == "line") {
      shape.points = shape.points.map(function (point) {
        return scale(point, factor)
      })
    }
    return shape
  })
}

function rotate(xy, about, angle) {
  var distance = getDistance(xy, about)
  var currentAngle = getAngle(xy, about)
  return [
    about[0] + Math.sin(currentAngle + angle) * distance,
    about[1] + Math.cos(currentAngle + angle) * distance,
  ]
}

function rotateShapes(shapes, about, angle) {
  return shapes.map(function (shape) {
    if (shape.type == "circle") {
      shape.center = rotate(shape.center, about, angle)
    }
    if (shape.type == "line") {
      shape.points = shape.points.map(function (point) {
        return rotate(point, about, angle)
      })
    }
    return shape
  })
}

function drawShip(ship) {
  var shapes = []
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
  var shapes = []

  // Orbit rings
  STATE.bodies.forEach(function (body) {
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
  STATE.bodies.forEach(function (body) {
    shapes.push({
      type: "circle",
      center: body.location,
      radius: getBodyRadius(body),
      strokeStyle: "#fff",
    })
  })

  // Ship
  shapes = shapes.concat(drawShip(STATE.ship))

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
var canvas = document.createElement("canvas")
canvas.width = window.innerWidth
canvas.height = window.innerHeight
document.body.appendChild(canvas)
var ctx = canvas.getContext("2d")

// Set event listeners
var KEYS = {}
document.onkeydown = function (e) {
  KEYS[e.key] = true
}
document.onkeyup = function (e) {
  KEYS[e.key] = false
}

var SYSTEM = {
  mass: 10000,
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
  var bodyToStartOn = STATE.bodies[_.random(STATE.bodies.length - 2) + 1]
  STATE.ship = {
    mass: 1,
    location: translate(bodyToStartOn.location, [
      10,
      -getBodyRadius(bodyToStartOn) + 10,
    ]),
    velocity: bodyToStartOn.velocity,
    angle: Math.PI,
    thrusting: false,
  }
}

reset()

setInterval(tick, 15)

render()
