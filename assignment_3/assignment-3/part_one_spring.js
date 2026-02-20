import {tiny, defs} from './examples/common.js';

const { vec3, vec4, color, Mat4, Shape, Material, Shader, Texture, Component } = tiny;

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
    this.p1 = p1; 
    this.p2 = p2; 
    this.ks = ks;
    this.kd = kd;
    this.rest_length = length;
  }
}

class Simulator {
  constructor() {
    this.particles = [];
    this.springs = [];
    this.integration_method = "symplectic";
    this.dt = 0.01;
    this.g = 9.8;
    this.ks_ground = 5000;
    this.kd_ground = 10;
    this.running = false;
    this.mu_ground = 0.05; 
    this.k_tangent = 0.0; 
  }

step() {
  console.log("FORCNING UPDATE V2"); 
  if (!this.running) return;

  // this is necessary if for forward euler if its too small then it mathmatically diverges
  // and it with actually introduce energy into the system causing the spline to go infinite
  const N = (this.integration_method === "euler") ? 10 : 1; // try 10, 20 if needed
  const h = this.dt / N;

  for (let sub = 0; sub < N; sub++) {
    // 1) Clear forces + gravity
    for (let p of this.particles) {
      p.force = vec3(0, -this.g * p.mass, 0);
    }

    // 2) Spring forces
    for (let s of this.springs) {
      const p1 = this.particles[s.p1];
      const p2 = this.particles[s.p2];
      const diff = p1.pos.minus(p2.pos);
      const l = diff.norm();
      if (l < 1e-6) continue;

      const d = diff.times(1 / l);
      const v_rel = p1.vel.minus(p2.vel);
      const f_mag = -s.ks * (l - s.rest_length) - s.kd * v_rel.dot(d);
      const f = d.times(f_mag);
      p1.force.add_by(f);
      p2.force.subtract_by(f);
    }

    // 3) Ground penalty 
    for (let p of this.particles) {
      if (p.pos[1] < 0) {
        const penetration = -p.pos[1];     // >= 0
        const v_n = p.vel[1];              // up is +
        
        // normal force
        let f_n = this.ks_ground * penetration - this.kd_ground * v_n;
        if (f_n < 0) f_n = 0;
        p.force[1] += f_n;

        // friction
        const v_t = vec3(p.vel[0], 0, p.vel[2]);   // tangential velocity
        const speed_t = v_t.norm();

        if (speed_t > 1e-6 && f_n > 0) {
          // Coulomb limit
          const f_max = this.mu_ground * f_n;

          // Direction opposing motion
          const dir = v_t.times(1 / speed_t);

          // Simple dynamic friction: apply max each step
          // (can feel "sticky"; add a scale if too strong)
          const f_fric = dir.times(-f_max);

          p.force.add_by(f_fric);
      }
      }
    }

    // 4) Integrate with step h
    for (let p of this.particles) {
      const a = p.force.times(1 / p.mass);

      if (this.integration_method === "euler") {
        p.pos.add_by(p.vel.times(h));
        p.vel.add_by(a.times(h));
      } else if (this.integration_method === "symplectic") {
        p.vel.add_by(a.times(h));
        p.pos.add_by(p.vel.times(h));
      } else if (this.integration_method === "verlet") {
        const temp = p.pos.copy();
        p.pos = p.pos.times(2).minus(p.pos_prev).plus(a.times(h * h));
        p.pos_prev = temp;
        p.vel = p.pos.minus(p.pos_prev).times(1 / h);
      }
    }
  }
}
}

// A dynamic polyline shape to draw springs
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
    super.draw(context, program_state, model_transform, material, "LINES");
  }
}

export const Part_one_spring_base = defs.Part_one_spring_base =
    class Part_one_spring_base extends Component {
      init() {
        this.hover = this.swarm = false;
        this.shapes = { 
          'box'  : new defs.Cube(),
          'ball' : new defs.Subdivision_Sphere( 4 ),
          'axis' : new defs.Axis_Arrows(),
          'spring_line': new Polyline()
        };

        const phong = new defs.Phong_Shader();
        const tex_phong = new defs.Textured_Phong();
        this.materials = {};
        this.materials.plastic = { shader: phong, ambient: .2, diffusivity: 1, specularity: .5, color: color( .9,.5,.9,1 ) }
        this.materials.metal   = { shader: phong, ambient: .2, diffusivity: 1, specularity:  1, color: color( .9,.5,.9,1 ) }
        this.materials.rgb = { shader: tex_phong, ambient: .5, texture: new Texture( "assets/rgb.jpg" ) }

        this.sim = new Simulator();
        this.parse_commands = this.parse_commands.bind(this);
        this.start = this.start.bind(this);
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

export class Part_one_spring extends Part_one_spring_base {
  render_animation( caller ) {
    super.render_animation( caller );
    const blue = color( 0,0,1,1 ), yellow = color( 1,1,0,1 ), red = color(1,0,0,1);

    let floor_transform = Mat4.translation(0, -0.01, 0).times(Mat4.scale(10, 0.01, 10));
    this.shapes.box.draw( caller, this.uniforms, floor_transform, { ...this.materials.plastic, color: yellow } );

    // Step the simulation
    if (this.sim.running) {
        const steps = Math.max(1, Math.floor(0.016 / this.sim.dt)); // Approx real-time playback
        for(let i=0; i<steps; i++) {
            this.sim.step();
        }
    }

    // Draw Particles
    for (let p of this.sim.particles) {
        let ball_transform = Mat4.translation(p.pos[0], p.pos[1], p.pos[2]).times(Mat4.scale(0.2, 0.2, 0.2));
        this.shapes.ball.draw( caller, this.uniforms, ball_transform, { ...this.materials.metal, color: blue } );
    }

    // Draw Springs
let line_points = [];
    for (let s of this.sim.springs) {
        line_points.push(this.sim.particles[s.p1].pos);
        line_points.push(this.sim.particles[s.p2].pos);
    }
    if (line_points.length > 0) {
        this.shapes.spring_line.set_points(line_points);
        // Force tiny-graphics to push the new vertex positions to the GPU
        this.shapes.spring_line.copy_onto_graphics_card(caller.context, ["position", "normal"], true); 
        this.shapes.spring_line.draw(caller, this.uniforms, Mat4.identity(), { ...this.materials.plastic, color: red });
    }
  }

  render_controls() {
    this.control_panel.innerHTML += "Part One:";
    this.new_line();
    this.key_triggered_button( "Config", [], this.parse_commands );
    this.new_line();
    this.key_triggered_button( "Run", [], this.start );
    this.new_line();
  }

  parse_commands() {
    const text = document.getElementById("input").value;
    const lines = text.split("\n").map(l => l.trim()).filter(l => l.length > 0 && !l.startsWith("#"));
    let applied = 0;

    try {
      for (const line of lines) {
        const w = line.split(/\s+/);

        if (w[0] === "create" && w[1] === "particles" && w.length === 3) {
          let n = parseInt(w[2]);
          this.sim.particles = new Array(n).fill(0).map(() => new Particle());
        } 
        else if (w[0] === "particle" && w.length === 9) {
          let i = parseInt(w[1]);
          this.sim.particles[i] = new Particle(parseFloat(w[2]), parseFloat(w[3]), parseFloat(w[4]), parseFloat(w[5]), parseFloat(w[6]), parseFloat(w[7]), parseFloat(w[8]));
        } 
        else if (w[0] === "all_velocities" && w.length === 4) {
          for (let p of this.sim.particles) {
            p.vel = vec3(parseFloat(w[1]), parseFloat(w[2]), parseFloat(w[3]));
          }
        } 
        else if (w[0] === "create" && w[1] === "springs" && w.length === 3) {
          let n = parseInt(w[2]);
          this.sim.springs = new Array(n);
        } 
        else if (w[0] === "link" && w.length === 7) {
          let sindex = parseInt(w[1]), p1 = parseInt(w[2]), p2 = parseInt(w[3]);
          let ks = parseFloat(w[4]), kd = parseFloat(w[5]), length = parseFloat(w[6]);
          
          if (length < 0) {
             length = this.sim.particles[p1].pos.minus(this.sim.particles[p2].pos).norm();
          }
          this.sim.springs[sindex] = new Spring(p1, p2, ks, kd, length);
        } 
        else if (w[0] === "integration" && w.length === 3) {
          this.sim.integration_method = w[1];
          this.sim.dt = parseFloat(w[2]);
        } 
        else if (w[0] === "ground" && w.length === 3) {
          this.sim.ks_ground = parseFloat(w[1]);
          this.sim.kd_ground = parseFloat(w[2]);
        } 
        else if (w[0] === "gravity" && w.length === 2) {
          this.sim.g = parseFloat(w[1]);
        } else {
            throw new Error(`Invalid command: ${line}`);
        }
        applied++;
      }
      document.getElementById("output").value = `Configured correctly. Applied ${applied} commands.`;
    } catch(e) {
      document.getElementById("output").value = `Error parsing config: ${e.message}`;
    }
  }

  start() {
    this.sim.running = true;
    
    // Set up prev positions for Verlet if needed
    for (let p of this.sim.particles) {
        p.pos_prev = p.pos.minus(p.vel.times(this.sim.dt));
    }
    document.getElementById("output").value = "Simulation Running...";
  }
}