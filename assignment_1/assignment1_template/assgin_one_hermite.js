
import {tiny, defs} from './examples/common.js';

// Pull these names into this module's scope for convenience:
const { vec3, vec4, color, Mat4, Shape, Material, Shader, Texture, Component } = tiny;

// TODO: you should implement the required classes here or in another file.

// hermite spline
class HermiteSpline { 
  constructor() {
    this.points = [];   // vec3
    this.tangents = []; // vec3 
  }
  
  clear() { // get rid of the spline
    this.points = []; 
    this.tangents = []; 
  }

  num_points() {  // get number of points
    return this.points.length; 
  }

  add_point(po,tan){ // add a point
    this.points.push(po); 
    this.tangents.push(tan); 
  }

  set_point(index, point){ // set a point
    if(index < 0 || index >= this.num_points()){
      throw new Error('set_point: index ${i} out of range'); 
    }
    this.points[index] = point; 
  }

  set_tangent(index, tanget){  // set a tanget
    if(index < 0 || index >= this.num_points()){
      throw new Error('set_tanget: index${i} out of range'); 
    }
    this.tangents[index] = tanget; 
  }

evaluate(t) {
  const n = this.num_points();
  if (n === 0) return vec3(0, 0, 0);
  if (n === 1) return this.points[0];

  // clamp global t
  t = Math.max(0, Math.min(1, t));

  const segments = n - 1;

  // map global t -> segment i and local u in [0,1]
  const scaled = t * segments;
  let i = Math.floor(scaled);
  if (i >= segments) i = segments - 1;
  const u = scaled - i;

  if (Math.abs(t - 0.5) < 1e-6) console.log("segments", segments, "i", i, "u", u);


  // IMPORTANT: tangents must be scaled because u = segments * t
  // if tangents are given in "global t" units, convert to "local u" units.
  const tangent_scale = 1 / segments;

  const p0 = this.points[i];
  const p1 = this.points[i + 1];
  const m0 = this.tangents[i].times(tangent_scale);
  const m1 = this.tangents[i + 1].times(tangent_scale);

  // just the standard cubic hermanitan basis
  const u2 = u * u, u3 = u2 * u;
  const h00 =  2*u3 - 3*u2 + 1;
  const h10 =      u3 - 2*u2 + u;
  const h01 = -2*u3 + 3*u2;
  const h11 =      u3 -   u2;

  return p0.times(h00)
    .plus(m0.times(h10))
    .plus(p1.times(h01))
    .plus(m1.times(h11));
}


  sample(samples_per_segment = 30) {
    const n = this.num_points();
    if (n < 2) return [];

    const segs = n - 1;
    const total = segs * samples_per_segment + 1;

    const pts = [];
    for (let k = 0; k < total; k++) {
      const t = k / (total - 1);
      pts.push(this.evaluate(t));
    }
    return pts;
  }

  arc_length(samples_per_segment = 60) {
    const pts = this.sample(samples_per_segment);
    if (pts.length < 2) return 0;

    let L = 0;
    for (let i = 1; i < pts.length; i++) {
      const d = pts[i].minus(pts[i-1]);
      L += Math.sqrt(d.dot(d));
    }
    return L;
  }

load_from_string(text) {
    const lines = text.trim().split("\n").map(l => l.trim()).filter(l => l.length);
    if (lines.length === 0) throw new Error("load: empty input");

    const n = parseInt(lines[0]);
    if (!Number.isFinite(n) || n < 0) throw new Error("load: first line must be integer n");
    if (lines.length !== 1 + n) throw new Error(`load: expected ${n} data lines, got ${lines.length - 1}`);

    this.clear();
    for (let i = 0; i < n; i++) {
      const w = lines[1+i].split(/\s+/);
      if (w.length !== 6) throw new Error(`load: line ${i+2} must have 6 numbers`);
      const cx = parseFloat(w[0]), cy = parseFloat(w[1]), cz = parseFloat(w[2]);
      const tx = parseFloat(w[3]), ty = parseFloat(w[4]), tz = parseFloat(w[5]);
      this.add_point(vec3(cx,cy,cz), vec3(tx,ty,tz));
    }
  }

export_to_string() {
    const n = this.num_points();
    let out = `${n}\n`;
    for (let i = 0; i < n; i++) {
      const p = this.points[i];
      const s = this.tangents[i];
      out += `${p[0]} ${p[1]} ${p[2]} ${s[0]} ${s[1]} ${s[2]}\n`;
    }
    return out.trim();
  }

arc_length_table(samples_per_segment = 60) {
  const pts = this.sample(samples_per_segment);
  if (pts.length < 2) {
    return { total: 0, table: [{ s: 0, t: 0 }] };
  }

  const N = pts.length;
  const s_vals = new Array(N);
  s_vals[0] = 0;

  let total = 0;
  for (let i = 1; i < N; i++) {
    const d = pts[i].minus(pts[i-1]);
    total += Math.sqrt(d.dot(d));
    s_vals[i] = total;
  }

  // t for each sample point (uniform in [0,1] by construction)
  const table = [];
  for (let i = 0; i < N; i++) {
    const t = i / (N - 1);
    table.push({ s: s_vals[i], t });
  }

  return { total, table };
}

}

// A dynamic polyline shape that draws sampled points as a LINE_STRIP.
class Polyline extends Shape {
  constructor(points = []) {
    super("position", "normal");
    this.set_points(points);
  }

  set_points(points) {
    this.arrays.position = points;
    this.arrays.normal = points.map(_ => vec3(0, 1, 0)); // dummy normals
    this.indices = [];
    for (let i = 0; i < points.length; i++) this.indices.push(i);
  }

  // Many tiny-graphics builds support passing a draw mode string as 5th arg.
  // If yours doesn't, tell me the error and I’ll adjust this to your exact version.
  draw(context, program_state, model_transform, material) {
    super.draw(context, program_state, model_transform, material, "LINE_STRIP");
  }
}


export
const Assign_one_hermite_base = defs.Assign_one_hermite_base =
    class Assign_one_hermite_base extends Component
    {                                          // **Assign_one_hermite_base** is a Scene that can be added to any display canvas.
                                               // This particular scene is broken up into two pieces for easier understanding.
                                               // The piece here is the base class, which sets up the machinery to draw a simple
                                               // scene demonstrating a few concepts.  A subclass of it, Assign_one_hermite,
                                               // exposes only the display() method, which actually places and draws the shapes,
                                               // isolating that code so it can be experimented with on its own.
      init()
      {
        console.log("init")

        // constructor(): Scenes begin by populating initial values like the Shapes and Materials they'll need.
        this.hover = this.swarm = false;
        // At the beginning of our program, load one of each of these shape
        // definitions onto the GPU.  NOTE:  Only do this ONCE per shape it
        // would be redundant to tell it again.  You should just re-use the
        // one called "box" more than once in display() to draw multiple cubes.
        // Don't define more than one blueprint for the same thing here.
        this.shapes = { 'box'  : new defs.Cube(),
          'ball' : new defs.Subdivision_Sphere( 4 ),
          'axis' : new defs.Axis_Arrows() };

        // *** Materials: ***  A "material" used on individual shapes specifies all fields
        // that a Shader queries to light/color it properly.  Here we use a Phong shader.
        // We can now tweak the scalar coefficients from the Phong lighting formulas.
        // Expected values can be found listed in Phong_Shader::update_GPU().
        const phong = new defs.Phong_Shader();
        const tex_phong = new defs.Textured_Phong();
        this.materials = {};
        this.materials.plastic = { shader: phong, ambient: .2, diffusivity: 1, specularity: .5, color: color( .9,.5,.9,1 ) }
        this.materials.metal   = { shader: phong, ambient: .2, diffusivity: 1, specularity:  1, color: color( .9,.5,.9,1 ) }
        this.materials.rgb = { shader: tex_phong, ambient: .5, texture: new Texture( "assets/rgb.jpg" ) }

        this.ball_location = vec3(1, 1, 1);
        this.ball_radius = 0.25;

        // TODO: you should create a Spline class instance

        this.spline = new HermiteSpline(); 
        this.spline_shape = new Polyline([]); 
        this.draw_spline = false; 

        this.parse_commands = this.parse_commands.bind(this); 
        this.update_scene = this.update_scene.bind(this); 
        this.load_spline = this.load_spline.bind(this); 
        this.export_spline = this.export_spline.bind(this); 
        this.preset_straight = this.preset_straight.bind(this); 
        this.preset_circle = this.preset_circle.bind(this); 
      }

      render_animation( caller )
      {                                                
        // display():  Called once per frame of animation.  We'll isolate out
        // the code that actually draws things into Assign_one_hermite, a
        // subclass of this Scene.  Here, the base class's display only does
        // some initial setup.

        // Setup -- This part sets up the scene's overall camera matrix, projection matrix, and lights:
        if( !caller.controls )
        { this.animated_children.push( caller.controls = new defs.Movement_Controls( { uniforms: this.uniforms } ) );
          caller.controls.add_mouse_controls( caller.canvas );

          // Define the global camera and projection matrices, which are stored in shared_uniforms.  The camera
          // matrix follows the usual format for transforms, but with opposite values (cameras exist as
          // inverted matrices).  The projection matrix follows an unusual format and determines how depth is
          // treated when projecting 3D points onto a plane.  The Mat4 functions perspective() or
          // orthographic() automatically generate valid matrices for one.  The input arguments of
          // perspective() are field of view, aspect ratio, and distances to the near plane and far plane.

          // !!! Camera changed here
          Shader.assign_camera( Mat4.look_at (vec3 (10, 10, 10), vec3 (0, 0, 0), vec3 (0, 1, 0)), this.uniforms );
        }
        this.uniforms.projection_transform = Mat4.perspective( Math.PI/4, caller.width/caller.height, 1, 100 );

        // *** Lights: *** Values of vector or point lights.  They'll be consulted by
        // the shader when coloring shapes.  See Light's class definition for inputs.
        const t = this.t = this.uniforms.animation_time/1000;
        const angle = Math.sin( t );

        // const light_position = Mat4.rotation( angle,   1,0,0 ).times( vec4( 0,-1,1,0 ) ); !!!
        // !!! Light changed here
        const light_position = vec4(20 * Math.cos(angle), 20,  20 * Math.sin(angle), 1.0);
        this.uniforms.lights = [ defs.Phong_Shader.light_source( light_position, color( 1,1,1,1 ), 1000000 ) ];

        // draw axis arrows.
        this.shapes.axis.draw(caller, this.uniforms, Mat4.identity(), this.materials.rgb);
      }
    }


export class Assign_one_hermite extends Assign_one_hermite_base
{                                                    // **Assign_one_hermite** is a Scene object that can be added to any display canvas.
                                                     // This particular scene is broken up into two pieces for easier understanding.
                                                     // See the other piece, My_Demo_Base, if you need to see the setup code.
                                                     // The piece here exposes only the display() method, which actually places and draws
                                                     // the shapes.  We isolate that code so it can be experimented with on its own.
                                                     // This gives you a very small code sandbox for editing a simple scene, and for
                                                     // experimenting with matrix transformations.
  render_animation( caller )
  {                                                // display():  Called once per frame of animation.  For each shape that you want to
    // appear onscreen, place a .draw() call for it inside.  Each time, pass in a
    // different matrix value to control where the shape appears.

    // Variables that are in scope for you to use:
    // this.shapes.box:   A vertex array object defining a 2x2x2 cube.
    // this.shapes.ball:  A vertex array object defining a 2x2x2 spherical surface.
    // this.materials.metal:    Selects a shader and draws with a shiny surface.
    // this.materials.plastic:  Selects a shader and draws a more matte surface.
    // this.lights:  A pre-made collection of Light objects.
    // this.hover:  A boolean variable that changes when the user presses a button.
    // shared_uniforms:  Information the shader needs for drawing.  Pass to draw().
    // caller:  Wraps the WebGL rendering context shown onscreen.  Pass to draw().

    // Call the setup code that we left inside the base class:
    super.render_animation( caller );

    /**********************************
     Start coding down here!!!!
     **********************************/
        // From here on down it's just some example shapes drawn for you -- freely
        // replace them with your own!  Notice the usage of the Mat4 functions
        // translation(), scale(), and rotation() to generate matrices, and the
        // function times(), which generates products of matrices.

    const blue = color( 0,0,1,1 ), yellow = color( 1,0.7,0,1 );

    const t = this.t = this.uniforms.animation_time/1000;

    // !!! Draw ground
    let floor_transform = Mat4.translation(0, 0, 0).times(Mat4.scale(10, 0.01, 10));
    this.shapes.box.draw( caller, this.uniforms, floor_transform, { ...this.materials.plastic, color: yellow } );

    // !!! Draw ball (for reference)
    let ball_transform = Mat4.translation(this.ball_location[0], this.ball_location[1], this.ball_location[2])
        .times(Mat4.scale(this.ball_radius, this.ball_radius, this.ball_radius));
    this.shapes.ball.draw( caller, this.uniforms, ball_transform, { ...this.materials.metal, color: blue } );

    // TODO: you should draw spline here.
    if (this.draw_spline && this.spline_shape.indices.length > 1) {
    this.spline_shape.draw(caller, this.uniforms, Mat4.identity(),
    { ...this.materials.metal, color: color(1,0,0,1) });
}
  }

  render_controls()
  {                                 // render_controls(): Sets up a panel of interactive HTML elements, including
    // buttons with key bindings for affecting this scene, and live info readouts.
    this.control_panel.innerHTML += "Assignment One:";
    this.new_line();
    this.key_triggered_button( "Parse Commands", [], this.parse_commands );
    this.new_line();
    this.key_triggered_button( "Draw", [], this.update_scene );
    this.new_line();
    this.key_triggered_button("Straight!", [], this.preset_straight); 
    this.new_line(); 
    this.key_triggered_button("Circle!", [], this.preset_circle); 
    this.new_line(); 
    this.key_triggered_button( "Load", [], this.load_spline );
    this.new_line();
    this.key_triggered_button( "Export", [], this.export_spline );
    this.new_line();

    /* Some code for your reference
    this.key_triggered_button( "Copy input", [ "c" ], function() {
      let text = document.getElementById("input").value;
      console.log(text);
      document.getElementById("output").value = text;
    } );
    this.new_line();
    this.key_triggered_button( "Relocate", [ "r" ], function() {
      let text = document.getElementById("input").value;
      const words = text.split(' ');
      if (words.length >= 3) {
        const x = parseFloat(words[0]);
        const y = parseFloat(words[1]);
        const z = parseFloat(words[2]);
        this.ball_location = vec3(x, y, z)
        document.getElementById("output").value = "success";
      }
      else {
        document.getElementById("output").value = "invalid input";
      }
    } );
     */
  }

  parse_commands() {
    //document.getElementById("output").value = "parse_commands";
    console.log("[DEBUG] PARSE COMMANDS PRESSED"); 
    // TODO 
  const text = document.getElementById("input").value;
  const lines = text.split("\n")
    .map(l => l.trim())
    .filter(l => l.length > 0 && !l.startsWith("#"));

  let applied = 0;

  try {
    for (const line of lines) {
      const w = line.split(/\s+/);

      // add point x y z sx sy sz
      if (w[0] === "add" && w[1] === "point" && w.length === 8) {
        const x = parseFloat(w[2]), y = parseFloat(w[3]), z = parseFloat(w[4]);
        const sx = parseFloat(w[5]), sy = parseFloat(w[6]), sz = parseFloat(w[7]);
        this.spline.add_point(vec3(x,y,z), vec3(sx,sy,sz));
        applied++;
        continue;
      }

      // set point i x y z
      if (w[0] === "set" && w[1] === "point" && w.length === 6) {
        const i = parseInt(w[2], 10);
        const x = parseFloat(w[3]), y = parseFloat(w[4]), z = parseFloat(w[5]);
        this.spline.set_point(i, vec3(x,y,z));
        applied++;
        continue;
      }

      // set tangent i sx sy sz
      if (w[0] === "set" && w[1] === "tangent" && w.length === 6) {
        const i = parseInt(w[2], 10);
        const sx = parseFloat(w[3]), sy = parseFloat(w[4]), sz = parseFloat(w[5]);
        this.spline.set_tangent(i, vec3(sx,sy,sz));
        applied++;
        continue;
      }

      // get_arc_length
      if (w[0] === "get_arc_length") {
        // Usage: get_arc_length [samples_per_segment] [rows]
        console.log("GET ARC LENGTH"); 
        let sps  = 60;
        let rows = 25;

        if (w.length >= 2) {
          sps = parseInt(w[1], 10);
          if (!Number.isFinite(sps) || sps < 2)
            throw new Error("get_arc_length: samples_per_segment must be int >= 2");
        }
        if (w.length >= 3) {
          rows = parseInt(w[2], 10);
          if (!Number.isFinite(rows) || rows < 2)
            throw new Error("get_arc_length: rows must be int >= 2");
        }
        if (w.length > 3)
          throw new Error("Usage: get_arc_length [samples_per_segment] [rows]");

        const { total, table } = this.spline.arc_length_table(sps);

        // Build output (reasonable number of rows) according to TA, we don't want to overload the machines
        const out = [];
        out.push(`Arc length parameterization (piecewise linear approx)`);
        out.push(`Arc length ≈ ${total.toFixed(6)}`);
        out.push(`Lookup table (s -> t):`);
        out.push(`s\t\tt`);

        const N = table.length;
        for (let r = 0; r < rows; r++) {
          const idx = Math.round(r * (N - 1) / (rows - 1));
          const { s, t } = table[idx];
          out.push(`${s.toFixed(6)}\t${t.toFixed(6)}`);
        }

        document.getElementById("output").value = out.join("\n");
        applied++;         // count it as a command
        return;            // dont print parsed ok
      }


      throw new Error(`Unknown/invalid command: "${line}"`);
    }

    document.getElementById("output").value = `Parsed OK. Applied ${applied} command(s). Control points: ${this.spline.num_points()}`;
  } catch (e) {
    document.getElementById("output").value = `Parse error after ${applied} command(s): ${e.message}`;
  }

  }

update_scene() { // Draw button
  console.log("[DEBUG] UPDATE SCENE PRESSED");

  try {
    const pts = this.spline.sample(40); // samples per segment
    if (pts.length < 2) {
      this.draw_spline = false;
      document.getElementById("output").value = "Nothing to draw (need at least 2 control points).";
      return;
    }
    // this.spline_shape.set_points(pts);
    // this.draw_spline = true;

    this.spline_shape = new Polyline(pts); 
    this.draw_spline = true; 
    document.getElementById("output").value = `Drew spline with ${pts.length} samples.`;
  } catch (e) {
    document.getElementById("output").value = `Draw error: ${e.message}`;
  }
}

load_spline() {
  console.log("[DEBUG] LOAD SPLINE PRESSED");

  const text = document.getElementById("input").value;

  try {
    this.spline.load_from_string(text);
    this.draw_spline = false; // don’t auto draw unless you want to
    document.getElementById("output").value = `Loaded spline. Control points: ${this.spline.num_points()}`;
  } catch (e) {
    document.getElementById("output").value = `Load error: ${e.message}`;
  }
}

export_spline() {
  console.log("[DEBUG] EXPORT SPLINE PRESSED");

  try {
    const out = this.spline.export_to_string();
    document.getElementById("output").value = out;
  } catch (e) {
    document.getElementById("output").value = `Export error: ${e.message}`;
  }
}

preset_straight() {
  // 4 control points in a straight line, equally spaced
  console.log("LINE"); 
  this.spline.clear();

  const y = 1.0;
  const p0 = vec3(-3, y, 0);
  const p1 = vec3(-1, y, 0);
  const p2 = vec3( 1, y, 0);
  const p3 = vec3( 3, y, 0);

  // For perfectly straight Hermite segments, set tangents equal to the per-segment delta
  const v = p1.minus(p0);   // (2,0,0) here

  this.spline.add_point(p0, v);
  this.spline.add_point(p1, v);
  this.spline.add_point(p2, v);
  this.spline.add_point(p3, v);

  // Immediately draw
  this.update_scene();
}

preset_circle() {
  console.log("CIRCLE"); // for debugging
  this.spline.clear();

  const y = 1.0;
  const R = 3.0;

  // More segments => looks like a circle
  const N = 80;

  const dtheta = (2 * Math.PI) / N;

  // This m_mag is the correct Hermite (derivative) magnitude for one segment in local u∈[0,1]
  const m_mag_local = 4 * Math.tan(dtheta / 4) * R;

  // IMPORTANT: your evaluate() multiplies tangents by (1/segments),
  // so store tangents scaled up by segments so that after scaling you get m_mag_local.
  const segments = N;                 // because we will add N+1 points (closed), so segments = (N+1)-1 = N
  const store_scale = segments;

  for (let k = 0; k < N; k++) {
    const theta = k * dtheta;

    const x = R * Math.cos(theta);
    const z = R * Math.sin(theta);

    // unit tangent direction on circle in xz-plane: (-sin, 0, cos)
    const tx = -Math.sin(theta) * m_mag_local * store_scale;
    const tz =  Math.cos(theta) * m_mag_local * store_scale;

    this.spline.add_point(vec3(x, y, z), vec3(tx, 0, tz));
  }

  // close loop (repeat first point + tangent)
  {
    const theta = 0;
    const x = R * Math.cos(theta);
    const z = R * Math.sin(theta);

    const tx = -Math.sin(theta) * m_mag_local * store_scale;
    const tz =  Math.cos(theta) * m_mag_local * store_scale;

    this.spline.add_point(vec3(x, y, z), vec3(tx, 0, tz));
  }

  this.update_scene();
}

}
