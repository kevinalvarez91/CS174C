import {tiny, defs} from './examples/common.js';

const { vec3, vec4, color, Mat4, Shape, Material, Shader, Texture, Component } = tiny;

// --- REUSED CLASSES (Simulator and Spline) ---
class Particle {
  constructor(mass=1, x=0, y=0, z=0, vx=0, vy=0, vz=0) {
    this.mass = mass;
    this.pos = vec3(x, y, z);
    this.vel = vec3(vx, vy, vz);
    this.force = vec3(0, 0, 0);
    this.pos_prev = vec3(x, y, z); 
  }
}

class Spring {
  constructor(p1, p2, ks, kd, length) {
    this.p1 = p1; this.p2 = p2; this.ks = ks; this.kd = kd; this.rest_length = length;
  }
}

class HermiteSpline { 
  constructor() {
    this.points = []; this.tangents = []; 
  }
  num_points() { return this.points.length; }
  add_point(po,tan){ this.points.push(po); this.tangents.push(tan); }

  evaluate(t) {
    const n = this.num_points();
    if (n === 0) return vec3(0, 0, 0);
    if (n === 1) return this.points[0];

    t = Math.max(0, Math.min(1, t));
    const segments = n - 1;
    const scaled = t * segments;
    let i = Math.floor(scaled);
    if (i >= segments) i = segments - 1;
    const u = scaled - i;

    const tangent_scale = 1 / segments;
    const p0 = this.points[i];
    const p1 = this.points[i + 1];
    const m0 = this.tangents[i].times(tangent_scale);
    const m1 = this.tangents[i + 1].times(tangent_scale);

    const u2 = u * u, u3 = u2 * u;
    const h00 =  2*u3 - 3*u2 + 1;
    const h10 =      u3 - 2*u2 + u;
    const h01 = -2*u3 + 3*u2;
    const h11 =      u3 -   u2;

    return p0.times(h00).plus(m0.times(h10)).plus(p1.times(h01)).plus(m1.times(h11));
  }

  sample(samples_per_segment = 30) {
    const n = this.num_points();
    if (n < 2) return [];
    const total = (n - 1) * samples_per_segment + 1;
    const pts = [];
    for (let k = 0; k < total; k++) {
      pts.push(this.evaluate(k / (total - 1)));
    }
    return pts;
  }
}

class Polyline extends Shape {
  constructor(points = []) {
    super("position", "normal");
    this.set_points(points);
  }
  set_points(points) {
    this.arrays.position = points;
    this.arrays.normal = points.map(_ => vec3(0, 1, 0));
    this.indices = points.map((_, i) => i);
  }
  draw(context, program_state, model_transform, material) {
    super.draw(context, program_state, model_transform, material, "LINE_STRIP");
  }
}

class Simulator {
  constructor() {
    this.particles = [];
    this.springs = [];
    this.integration_method = "symplectic";
    this.dt = 0.005;
    this.g = 9.8;
    this.ks_ground = 10000;
    this.kd_ground = 100;
    this.running = true;
  }

  step() {
    if (!this.running) return;
    for (let p of this.particles) p.force = vec3(0, -this.g * p.mass, 0);

    for (let s of this.springs) {
      let p1 = this.particles[s.p1];
      let p2 = this.particles[s.p2];
      let diff = p1.pos.minus(p2.pos);
      let l = diff.norm();
      if (l < 1e-6) continue;
      
      let d = diff.normalized();
      let v_rel = p1.vel.minus(p2.vel);
      let f_mag = -s.ks * (l - s.rest_length) - s.kd * v_rel.dot(d);
      let f = d.times(f_mag);
      
      p1.force.add_by(f);
      p2.force.subtract_by(f);
    }

    for (let p of this.particles) {
      if (p.pos[1] < 0) {
        let f_penalty = -this.ks_ground * p.pos[1] - this.kd_ground * p.vel[1];
        p.force[1] += Math.max(0, f_penalty); 
      }
    }

    // Integrate (skip particle 0 as it's driven by the spline)
    for (let i = 1; i < this.particles.length; i++) {
      let p = this.particles[i];
      let a = p.force.times(1 / p.mass);
      p.vel.add_by(a.times(this.dt));
      p.pos.add_by(p.vel.times(this.dt));
    }
  }
}

// --- MAIN BASE COMPONENT ---
export const Part_two_chain_base = defs.Part_two_chain_base =
    class Part_two_chain_base extends Component {
      init() {
        this.hover = this.swarm = false;
        this.shapes = { 
            'box'  : new defs.Cube(),
            'ball' : new defs.Subdivision_Sphere( 4 ),
            'axis' : new defs.Axis_Arrows(),
            'spline': new Polyline(),
            'springs': new Polyline()
        };

        const phong = new defs.Phong_Shader();
        const tex_phong = new defs.Textured_Phong();
        this.materials = {};
        this.materials.plastic = { shader: phong, ambient: .2, diffusivity: 1, specularity: .5, color: color( .9,.5,.9,1 ) }
        this.materials.metal   = { shader: phong, ambient: .2, diffusivity: 1, specularity:  1, color: color( .9,.5,.9,1 ) }
        this.materials.rgb = { shader: tex_phong, ambient: .5, texture: new Texture( "assets/rgb.jpg" ) }

        // Initialize Simulator
        this.sim = new Simulator();
        let num_particles = 8;
        for (let i = 0; i < num_particles; i++) {
            this.sim.particles.push(new Particle(1.0, 0, 8 - i * 0.8, 0, 0, 0, 0));
        }
        for (let i = 0; i < num_particles - 1; i++) {
            this.sim.springs.push(new Spring(i, i+1, 500, 15, 1.0));
        }

        // Initialize Trajectory Spline
        this.spline = new HermiteSpline();
        this.spline.add_point(vec3(-5, 6, 0), vec3(8, 0, 8));
        this.spline.add_point(vec3(-2, 8, 3), vec3(8, 0, -8));
        this.spline.add_point(vec3(2, 6, -3), vec3(8, 0, 8));
        this.spline.add_point(vec3(5, 8, 0), vec3(8, 0, -8));
        this.shapes.spline.set_points(this.spline.sample(40));
      }

      render_animation( caller ) {
        if(!caller.controls) { 
            this.animated_children.push( caller.controls = new defs.Movement_Controls( { uniforms: this.uniforms } ) );
            caller.controls.add_mouse_controls( caller.canvas );
            Shader.assign_camera( Mat4.look_at (vec3 (15, 15, 15), vec3 (0, 0, 0), vec3 (0, 1, 0)), this.uniforms );
        }
        this.uniforms.projection_transform = Mat4.perspective( Math.PI/4, caller.width/caller.height, 1, 100 );

        const t = this.t = this.uniforms.animation_time/1000;
        const angle = Math.sin( t );
        const light_position = vec4(20 * Math.cos(angle), 20,  20 * Math.sin(angle), 1.0);
        this.uniforms.lights = [ defs.Phong_Shader.light_source( light_position, color( 1,1,1,1 ), 1000000 ) ];
        this.shapes.axis.draw(caller, this.uniforms, Mat4.identity(), this.materials.rgb);
      }
    }

export class Part_two_chain extends Part_two_chain_base {
  render_animation( caller ) {
    super.render_animation( caller );
    const blue = color( 0,0,1,1 ), yellow = color( 0.7,1,0,1 ), gray = color(0.5, 0.5, 0.5, 1);

    // Draw Ground
    let floor_transform = Mat4.translation(0, -0.01, 0).times(Mat4.scale(10, 0.01, 10));
    this.shapes.box.draw( caller, this.uniforms, floor_transform, { ...this.materials.plastic, color: yellow } );

    // Draw Spline
    this.shapes.spline.draw(caller, this.uniforms, Mat4.identity(), { ...this.materials.plastic, color: gray });

    // Enforce top particle sinusoidal trajectory on spline
    let spline_t = 0.5 + 0.5 * Math.sin(this.t); 
    this.sim.particles[0].pos = this.spline.evaluate(spline_t);
    this.sim.particles[0].vel = vec3(0,0,0); // Let the manual update handle top particle's placement

    // Step Simulation
    const steps = 4; // Multiple steps for stability
    for(let i=0; i<steps; i++) {
        this.sim.step();
    }

    // Draw Particles
    for (let p of this.sim.particles) {
        let ball_transform = Mat4.translation(p.pos[0], p.pos[1], p.pos[2]).times(Mat4.scale(0.3, 0.3, 0.3));
        this.shapes.ball.draw( caller, this.uniforms, ball_transform, { ...this.materials.metal, color: blue } );
    }

    // Draw Springs (as continuous line strip)
let line_points = this.sim.particles.map(p => p.pos);
    if (line_points.length > 0) {
        this.shapes.springs.set_points(line_points);
        // Force tiny-graphics to push the new vertex positions to the GPU
        this.shapes.springs.copy_onto_graphics_card(caller.context, ["position", "normal"], true);
        this.shapes.springs.draw(caller, this.uniforms, Mat4.identity(), { ...this.materials.plastic, color: gray });
    }
  }

  render_controls() {
    this.control_panel.innerHTML += "Part Two: Viscoelastic Chain (Autoplaying)";
    this.new_line();
  }
}