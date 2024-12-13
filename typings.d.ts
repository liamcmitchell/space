import { vec2, mat3 } from "gl-matrix"

declare global {
  interface Window {
    twgl: typeof import("twgl.js")
    glMatrix: typeof import("gl-matrix")
  }

  type Vec2 = vec2

  type Vec4 = [number, number, number, number]

  interface Thruster {
    location: vec2
    angle: number
    force: number
    on: boolean
    noise: (on: boolean) => void
    key: string
    touch: string
    transform: mat3
  }

  type BodyType = "circle" | "poly"

  interface DynamicBody {
    type: BodyType
    fixed: boolean
    radius: number
    mass: number
    location: Vec2
    velocity: Vec2
    force: Vec2
    inertia: number
    angle: number
    angularVelocity: number
    torque: number
    points?: Vec2[]
    absolutePoints?: Vec2[]
    absolutePointsTime?: number
    orbitCenter?: Vec2
    orbitRadius?: number
    landed?: boolean
  }

  interface Ship extends DynamicBody {
    thrusters: Thruster[]
    health: number
    destroyed: boolean
    destroyedFor: number
  }

  interface System {
    mass: number
    orbitRadius: number
    children?: System[]
    orbitVelocity?: number
  }

  interface DebugPoint {
    color: Vec4
    origin: Vec2
    offset?: Vec2
  }

  interface State {
    running: boolean
    rendered: boolean
    time: number
    timeInc: number
    zoom: number
    idealZoom: number
    viewRadius: number
    width: number
    height: number
    bodies: DynamicBody[]
    ship: Ship
    debugPoints?: DebugPoint[]
    pointsRendered: number
    fps: number
    lastRender: number
  }

  interface Collision {
    point: Vec2
    normal: Vec2
    depth: number
  }
}

export {}
