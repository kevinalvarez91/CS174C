import {tiny, defs} from './examples/common.js';

// Pull these names into this module's scope for convenience:
const { vec3, vec4, color, Mat4, Shape, Material, Shader, Texture, Component } = tiny;

// TODO: you should implement the required classes here or in another file.

const shapes = {
	'sphere': new defs.Subdivision_Sphere( 5 ),
};
export
const Articulated_Human =
class Articulated_Human {
	constructor() {
		const sphere_shape = shapes.sphere;
		// torso node
		const torso_transform = Mat4.scale(1, 2.5, 0.5);
		this.torso_node = new Node("torso", sphere_shape, torso_transform);
		// root->torso
		const root_location = Mat4.translation(-2, 5, 0);
		this.root = new Arc("root", null, this.torso_node, root_location);
		// head node
		let head_transform = Mat4.scale(.6, .6, .6);
		head_transform.pre_multiply(Mat4.translation(0, .6, 0));
		this.head_node = new Node("head", sphere_shape, head_transform);
		// torso->neck->head
		const neck_location = Mat4.translation(0, 2.5, 0);
		this.neck = new Arc("neck", this.torso_node, this.head_node,
				neck_location);
		this.torso_node.children_arcs.push(this.neck);
		// right upper arm node
		let ru_arm_transform = Mat4.scale(1.2, .2, .2);
		ru_arm_transform.pre_multiply(Mat4.translation(1.2, 0, 0));
		this.ru_arm_node = new Node("ru_arm", sphere_shape, ru_arm_transform);
		// torso->r_shoulder->ru_arm
		const r_shoulder_location = Mat4.translation(0.6, 2, 0);
		this.r_shoulder = new Arc("r_shoulder", this.torso_node, this.ru_arm_node,
				r_shoulder_location);
		this.torso_node.children_arcs.push(this.r_shoulder)
			// right lower arm node
			let rl_arm_transform = Mat4.scale(1, .2, .2);
		rl_arm_transform.pre_multiply(Mat4.translation(1, 0, 0));
		this.rl_arm_node = new Node("rl_arm", sphere_shape, rl_arm_transform);
		// ru_arm->r_elbow->rl_arm
		const r_elbow_location = Mat4.translation(2.4, 0, 0);
		this.r_elbow = new Arc("r_elbow", this.ru_arm_node, this.rl_arm_node,
				r_elbow_location);
		this.ru_arm_node.children_arcs.push(this.r_elbow)
			// right hand node
			let r_hand_transform = Mat4.scale(.4, .3, .2);
		r_hand_transform.pre_multiply(Mat4.translation(0.4, 0, 0));
		this.r_hand_node = new Node("r_hand", sphere_shape, r_hand_transform);
		// rl_arm->r_wrist->r_hand
		const r_wrist_location = Mat4.translation(2, 0, 0);
		this.r_wrist = new Arc("r_wrist", this.rl_arm_node, this.r_hand_node,
				r_wrist_location);
		this.rl_arm_node.children_arcs.push(this.r_wrist)
	}
	draw(webgl_manager, uniforms, material) {
		this.matrix_stack = [];
		this._rec_draw(this.root, Mat4.identity(), webgl_manager, uniforms,
				material);
	}
	_rec_draw(arc, matrix, webgl_manager, uniforms, material) {
		if (arc !== null) {
			const L = arc.location_matrix;
			const A = arc.articulation_matrix;
			matrix.post_multiply(L.times(A));
			this.matrix_stack.push(matrix.copy());
			const node = arc.child_node;
			const T = node.transform_matrix;
			matrix.post_multiply(T);
			node.shape.draw(webgl_manager, uniforms, matrix, material);
			matrix = this.matrix_stack.pop();
			for (const next_arc of node.children_arcs) {
				this.matrix_stack.push(matrix.copy());
				this._rec_draw(next_arc, matrix, webgl_manager, uniforms,
						material);
				matrix = this.matrix_stack.pop();
			}
		}
	}
	debug(arc=null) {
		if (arc === null)
			arc = this.root;
		if (arc !== this.root) {
			arc.articulation_matrix =
				arc.articulation_matrix.times(Mat4.rotation(0.02, 0, 0, 1));
		}
		const node = arc.child_node;
		for (const next_arc of node.children_arcs) {
			this.debug(next_arc);
		}
	}
	// Inside the Articulated_Human class
	solve_ik(target_pos) {
		// Joints in order
		const joints = [this.r_shoulder, this.r_elbow, this.r_wrist];

		// Per-spec DOFs:
		// shoulder: x,y,z  (3)
		// elbow:    x,y    (2)
		// wrist:    y,z    (2)
		const axes_by_joint = new Map([
				[this.r_shoulder, [vec3(1,0,0), vec3(0,1,0), vec3(0,0,1)]],
				[this.r_elbow,    [vec3(1,0,0), vec3(0,1,0)]],
				[this.r_wrist,    [vec3(0,1,0), vec3(0,0,1)]],
		]);

		const end_effector_pos = this.get_end_effector_position();
		const dx_vec = target_pos.minus(end_effector_pos);

		if (dx_vec.norm() < 0.01) return;

		// Build Jacobian: 3 x N
		const J = [[], [], []];

		for (const joint of joints) {
			const joint_pos = this.get_joint_world_pos(joint);
			const r = end_effector_pos.minus(joint_pos);

			const axes = axes_by_joint.get(joint);
			for (const axis_local of axes) {
				const axis_world = this.get_joint_world_axis(joint, axis_local);
				const col = axis_world.cross(r); // (a x r)

				J[0].push(col[0]);
				J[1].push(col[1]);
				J[2].push(col[2]);
			}
		}

		// Solve J * dtheta ~= dx
		const dtheta = solve_linear_system(J, [dx_vec[0], dx_vec[1], dx_vec[2]]);

		// Apply with a small gain for stability
		const gain = 0.025;
		let k = 0;

		// shoulder: x,y,z
		{
			const rx = gain * dtheta[k++], ry = gain * dtheta[k++], rz = gain * dtheta[k++];
			this.r_shoulder.articulation_matrix = this.r_shoulder.articulation_matrix
				.times(Mat4.rotation(rx, 1,0,0))
				.times(Mat4.rotation(ry, 0,1,0))
				.times(Mat4.rotation(rz, 0,0,1));
		}

		// elbow: x,y
		{
			const rx = gain * dtheta[k++], ry = gain * dtheta[k++];
			this.r_elbow.articulation_matrix = this.r_elbow.articulation_matrix
				.times(Mat4.rotation(rx, 1,0,0))
				.times(Mat4.rotation(ry, 0,1,0));
		}

		// wrist: y,z
		{
			const ry = gain * dtheta[k++], rz = gain * dtheta[k++];
			this.r_wrist.articulation_matrix = this.r_wrist.articulation_matrix
				.times(Mat4.rotation(ry, 0,1,0))
				.times(Mat4.rotation(rz, 0,0,1));
		}
	}


	// Inside Articulated_Human class in assignment2.js
	get_arc_world_matrix(target_arc) {
		let matrix = Mat4.identity();
		const find_matrix = (arc, current_matrix) => {
			if (!arc) return null;
			const L = arc.location_matrix;
			const A = arc.articulation_matrix;
			const new_matrix = current_matrix.times(L).times(A);

			if (arc === target_arc) return new_matrix;

			const node = arc.child_node;
			for (const next_arc of node.children_arcs) {
				const found = find_matrix(next_arc, new_matrix);
				if (found) return found;
			}
			return null;
		};
		return find_matrix(this.root, matrix);
	}

	get_end_effector_position() {
		// World matrix up to wrist joint (includes wrist L and wrist A)
		const Mwrist = this.get_arc_world_matrix(this.r_wrist);
		if (!Mwrist) return vec3(0,0,0);

		// Include hand node's transform (scale + pre-translate)
		const Thand = this.r_hand_node.transform_matrix;

		// Tip in hand-local coords: your pre_multiply translation is 0.4,
		// so a reasonable "tip" is about x = 0.8 from the hand's local origin.
		const Mtip = Mwrist.times(Thand).times(Mat4.translation(0.8, 0, 0));

		const p4 = Mtip.times(vec4(0,0,0,1));
		return vec3(p4[0], p4[1], p4[2]);
	}


	// Inside Articulated_Human class

	// Helper to find the world-space matrix of a specific Arc (joint)
	get_arc_world_matrix(target_arc) {
		let world_matrix = Mat4.identity();

		// Recursive helper to traverse the tree and find the target arc
		const find_matrix = (current_arc, current_matrix) => {
			if (!current_arc) return null;

			const L = current_arc.location_matrix;
			const A = current_arc.articulation_matrix;
			// Apply this arc's transformations
			const new_matrix = current_matrix.times(L).times(A);

			if (current_arc === target_arc) {
				return new_matrix;
			}

			const node = current_arc.child_node;
			for (const next_arc of node.children_arcs) {
				const found = find_matrix(next_arc, new_matrix);
				if (found) return found;
			}
			return null;
		};

		return find_matrix(this.root, world_matrix);
	}

	// Returns the world-space position of a joint
	get_joint_world_pos(joint_arc) {
		const m = this.get_arc_world_matrix(joint_arc);
		return m.times(vec4(0, 0, 0, 1)).to3();
	}

	// Returns the world-space direction of a local rotation axis (e.g., [1,0,0])
	get_joint_world_axis(joint_arc, local_axis) {
		// local_axis is a vec3 like vec3(1,0,0)
		const m = this.get_arc_world_matrix(joint_arc);

		// Transform direction by rotation part only (w=0 ignores translation)
		const a4 = vec4(local_axis[0], local_axis[1], local_axis[2], 0);
		const w4 = m.times(a4);

		return vec3(w4[0], w4[1], w4[2]).normalized();
	}


	// Updated version of the end effector helper
	get_end_effector_position() {
		// We want the tip of the hand. 
		// In your constructor, r_hand_node has a pre_multiply(Mat4.translation(0.4, 0, 0))
		// and a scale(.4, .3, .2). The tip is roughly at local x = 0.8
		const world_matrix = this.get_arc_world_matrix(this.r_wrist);
		if (!world_matrix) return vec3(0,0,0);
		return world_matrix.times(Mat4.translation(0.8, 0, 0)).times(vec4(0, 0, 0, 1)).to3();
	}


}


class Node {
	constructor(name, shape, transform) {
		this.name = name;
		this.shape = shape;
		this.transform_matrix = transform;
		this.children_arcs = [];
	}
}

class Arc {
	constructor(name, parent, child, location) {
		this.name = name;
		this.parent_node = parent;
		this.child_node = child;
		this.location_matrix = location;
		this.articulation_matrix = Mat4.identity();
	}
}


// -------------------------
// Hermite spline + polyline
// -------------------------

// Minimal Hermite spline (same math as in assgin_one_hermite.js) but without any textbox I/O.
class HermiteSpline {
	constructor() {
		this.points = [];   // vec3
		this.tangents = []; // vec3
	}

	clear() { this.points = []; this.tangents = []; }
	num_points() { return this.points.length; }

	add_point(p, m) { this.points.push(p); this.tangents.push(m); }

	evaluate(t) {
		const n = this.num_points();
		if (n === 0) return vec3(0, 0, 0);
		if (n === 1) return this.points[0];

		// Clamp global t to [0,1]
		t = Math.max(0, Math.min(1, t));

		const segments = n - 1;

		// Map global t -> segment i and local u in [0,1]
		const scaled = t * segments;
		let i = Math.floor(scaled);
		if (i >= segments) i = segments - 1;
		const u = scaled - i;

		// Tangents are stored in "global t" units; convert to "local u" units.
		const tangent_scale = 1 / segments;

		const p0 = this.points[i];
		const p1 = this.points[i + 1];
		const m0 = this.tangents[i].times(tangent_scale);
		const m1 = this.tangents[i + 1].times(tangent_scale);

		// Standard cubic Hermite basis
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
}

// A dynamic polyline shape that draws sampled points as a LINE_STRIP.
class Polyline extends Shape {
	constructor(points = []) {
		super("position", "normal");
		this.set_points(points);
	}

	set_points(points) {
		this.arrays.position = points;
		this.arrays.normal = points.map(_ => vec3(0, 1, 0));
		this.indices = [];
		for (let i = 0; i < points.length; i++) this.indices.push(i);
	}

	draw(context, program_state, model_transform, material) {
		super.draw(context, program_state, model_transform, material, "LINE_STRIP");
	}
}

// helper function 
// Add this helper function outside the classes in assignment2.js
function solve_linear_system(J, dx) {
	// Damped least squares: dtheta = J^T (J J^T + λ^2 I)^-1 dx
	const lambda = 0.1;

	// Build J^T (N x 3)
	const JT = [];
	for (let i = 0; i < J[0].length; i++) {
		JT[i] = [J[0][i], J[1][i], J[2][i]];
	}

	// Compute JJT = (3x3)
	const JJT = [[0,0,0],[0,0,0],[0,0,0]];
	for (let i = 0; i < 3; i++) {
		for (let j = 0; j < 3; j++) {
			let sum = 0;
			for (let k = 0; k < J[0].length; k++) sum += J[i][k] * JT[k][j];
			JJT[i][j] = sum;
		}
	}

	// Add damping λ^2 I
	JJT[0][0] += lambda*lambda;
	JJT[1][1] += lambda*lambda;
	JJT[2][2] += lambda*lambda;

	// Invert 3x3
	const det =
		JJT[0][0]*(JJT[1][1]*JJT[2][2] - JJT[1][2]*JJT[2][1]) -
		JJT[0][1]*(JJT[1][0]*JJT[2][2] - JJT[1][2]*JJT[2][0]) +
		JJT[0][2]*(JJT[1][0]*JJT[2][1] - JJT[1][1]*JJT[2][0]);

	if (Math.abs(det) < 1e-10) return new Array(J[0].length).fill(0);

	const invDet = 1.0 / det;
	const inv = [
		[
			(JJT[1][1]*JJT[2][2] - JJT[1][2]*JJT[2][1]) * invDet,
		(JJT[0][2]*JJT[2][1] - JJT[0][1]*JJT[2][2]) * invDet,
		(JJT[0][1]*JJT[1][2] - JJT[0][2]*JJT[1][1]) * invDet
		],
		[
			(JJT[1][2]*JJT[2][0] - JJT[1][0]*JJT[2][2]) * invDet,
		(JJT[0][0]*JJT[2][2] - JJT[0][2]*JJT[2][0]) * invDet,
		(JJT[0][2]*JJT[1][0] - JJT[0][0]*JJT[1][2]) * invDet
		],
		[
			(JJT[1][0]*JJT[2][1] - JJT[1][1]*JJT[2][0]) * invDet,
		(JJT[0][1]*JJT[2][0] - JJT[0][0]*JJT[2][1]) * invDet,
		(JJT[0][0]*JJT[1][1] - JJT[0][1]*JJT[1][0]) * invDet
		]
	];

		// temp = inv * dx  (3)
		const temp = [0,0,0];
		for (let i = 0; i < 3; i++) {
			temp[i] = inv[i][0]*dx[0] + inv[i][1]*dx[1] + inv[i][2]*dx[2];
		}

		// dtheta = JT * temp  (N)
		const dtheta = new Array(J[0].length).fill(0);
		for (let i = 0; i < J[0].length; i++) {
			dtheta[i] = JT[i][0]*temp[0] + JT[i][1]*temp[1] + JT[i][2]*temp[2];
		}

		return dtheta;
}


export
const Assignment2_base = defs.Assignment2_base =
class Assignment2_base extends Component
{                                          
	// **My_Demo_Base** is a Scene that can be added to any display canvas.
	// This particular scene is broken up into two pieces for easier understanding.
	// The piece here is the base class, which sets up the machinery to draw a simple
	// scene demonstrating a few concepts.  A subclass of it, Assignment2,
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
		const basic = new defs.Basic_Shader();
		const phong = new defs.Phong_Shader();
		const tex_phong = new defs.Textured_Phong();
		this.materials = {};
		this.materials.plastic = { shader: phong, ambient: .2, diffusivity: 1, specularity: .5, color: color( .9,.5,.9,1 ) }
		this.materials.metal   = { shader: phong, ambient: .2, diffusivity: 1, specularity:  1, color: color( .9,.5,.9,1 ) }
		this.materials.rgb = { shader: tex_phong, ambient: .5, texture: new Texture( "assets/rgb.jpg" ) }

		this.ball_location = vec3(1, 1, 1);
		this.ball_radius = 0.25;

		// TODO: you should create a Spline class instance

		// Create a hardcoded Hermite spline and a polyline mesh for it.
		this.spline = new HermiteSpline();
		this.human = new Articulated_Human(); 

		// Control points + tangents for "8" (tangents are (dx,dy,0) in world coords)
		// Figure-8 with smoother circular loops
		const blackboard_z = -0.9; 
this.spline.clear();

this.spline.add_point(vec3(-3.15 + 6.0,  0.146 + 6.0,  blackboard_z), vec3(-14.8,   8.442, 0));
this.spline.add_point(vec3(-4.29 + 6.0,  1.32 + 6.0,   blackboard_z), vec3( 0.0,   12.42,  0));
this.spline.add_point(vec3(-3.146 + 6.0, 2.276 + 6.0,  blackboard_z), vec3(10.43, -0.342,  0));
this.spline.add_point(vec3(-1.92 + 6.0,  1.37 + 6.0,   blackboard_z), vec3(0.153, -13.37,  0));
this.spline.add_point(vec3(-3.152 + 6.0, 0.1505 + 6.0, blackboard_z), vec3(-14.09, -8.14,  0));
this.spline.add_point(vec3(-4.27 + 6.0, -1.08 + 6.0,   blackboard_z), vec3(1.08,  -13.14,  0));
this.spline.add_point(vec3(-3.065 + 6.0,-1.957 + 6.0,  blackboard_z), vec3(10.7,  -0.144,  0));
this.spline.add_point(vec3(-1.887 + 6.0,-1.082 + 6.0,  blackboard_z), vec3(-0.162, 14.15,  0));

// Closing point (same as first)
this.spline.add_point(vec3(-3.15 + 6.0,  0.146 + 6.0,  blackboard_z), vec3(-14.8, 8.442, 0));



		const spline_pts = this.spline.sample(400); // samples per segment
		this.shapes.spline = new Polyline(spline_pts);
		this.draw_spline = (spline_pts.length > 1);



		this.hand_reached_board = false; 
		this.initial_target = vec3(3.0, 6.0, -0.9); // Adjust to match your blackboard position

		this.spline_u = 0.0; 
		this.spline_speed = 0.1;

	}

	render_animation( caller )
	{                                                // display():  Called once per frame of animation.  We'll isolate out
							 // the code that actually draws things into Assignment2, a
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
			// TODO: you can change the camera as needed.
			Shader.assign_camera( Mat4.look_at (vec3 (5, 8, 15), vec3 (0, 5, 0), vec3 (0, 1, 0)), this.uniforms );
		}
		this.uniforms.projection_transform = Mat4.perspective( Math.PI/4, caller.width/caller.height, 1, 100 );

		// *** Lights: *** Values of vector or point lights.  They'll be consulted by
		// the shader when coloring shapes.  See Light's class definition for inputs.
		const t = this.t = this.uniforms.animation_time/1000;

		// const light_position = Mat4.rotation( angle,   1,0,0 ).times( vec4( 0,-1,1,0 ) ); !!!
		// !!! Light changed here
		const light_position = vec4(20, 20, 20, 1.0);
		this.uniforms.lights = [ defs.Phong_Shader.light_source( light_position, color( 1,1,1,1 ), 1000000 ) ];

		// draw axis arrows.
		this.shapes.axis.draw(caller, this.uniforms, Mat4.identity(), this.materials.rgb);
	}
}


export class Assignment2 extends Assignment2_base
{                                                    
	// **Assignment2** is a Scene object that can be added to any display canvas.
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

		const blue = color( 0,0,1,1 ), yellow = color( 1,0.7,0,1 ), 
		      wall_color = color( 0.7, 1.0, 0.8, 1 ), 
		      blackboard_color = color( 0.2, 0.2, 0.2, 1 );


		const t = this.t = this.uniforms.animation_time/1000;

		// !!! Draw ground
		let floor_transform = Mat4.translation(0, 0, 0).times(Mat4.scale(10, 0.01, 10));
		this.shapes.box.draw( caller, this.uniforms, floor_transform, { ...this.materials.plastic, color: yellow } );


		// Draw the hardcoded spline as a red polyline (static).
		if (this.draw_spline && this.shapes.spline && this.shapes.spline.indices.length > 1) {
			this.shapes.spline.draw(
					caller,
					this.uniforms,
					Mat4.identity(),
					{ ...this.materials.metal, color: color(1, 0, 0, 1) }
					);
		}
		// TODO: you should draw scene here.
		// TODO: you can change the wall and board as needed.

		// --- ADD THIS IK LOGIC HERE ---
		if (!this.hand_reached_board) {
			// Phase 1: move to the board target
			const end_pos = this.human.get_end_effector_position();
			const distance = this.initial_target.minus(end_pos).norm();

			this.human.solve_ik(this.initial_target);

			if (distance < 0.1) {
				this.hand_reached_board = true;

				// OPTIONAL: start tracing from the center point of the spline
				// (center intersection is t=0 if you built it that way, otherwise just start at 0)
				this.spline_u = 0.0;
			}
		} else {
			// Phase 2: trace spline forever
			const dt = this.uniforms.animation_delta_time / 1000; // seconds since last frame

			// advance u, wrap into [0,1)
			this.spline_u = (this.spline_u + this.spline_speed * dt) % 1.0;

			// evaluate spline at u
			const target_on_spline = this.spline.evaluate(this.spline_u);

			// move hand toward that moving target
			this.human.solve_ik(target_on_spline);
		}
		// ------------------------------

		this.human.draw(caller, this.uniforms, this.materials.plastic);  
		let wall_transform = Mat4.translation(0, 5, -1.2).times(Mat4.scale(6, 5, 0.1));
		this.shapes.box.draw( caller, this.uniforms, wall_transform, { ...this.materials.plastic, color: wall_color } );
		let board_transform = Mat4.translation(3, 6, -1).times(Mat4.scale(2.5, 2.5, 0.1));
		this.shapes.box.draw( caller, this.uniforms, board_transform, { ...this.materials.plastic, color: blackboard_color } );
	}

	render_controls()
	{                                 
		// render_controls(): Sets up a panel of interactive HTML elements, including
		// buttons with key bindings for affecting this scene, and live info readouts.
		this.control_panel.innerHTML += "Assignment 2: IK Engine";
		this.new_line();    
		// TODO: You can add your button events for debugging. (optional)
		this.key_triggered_button( "Debug", [ "Shift", "D" ], null );
		this.new_line();
	}
}