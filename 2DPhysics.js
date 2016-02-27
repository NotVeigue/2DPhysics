
// Global Values
g_rbId = -1;

function Camera2D(ctx, w, h, scale, posX, posY)
{
	var o = this;

	o.ctx = ctx;
	o.viewportWidth = w;
	o.viewportHeight = h;
	o.scaleFactor = scale || 1;
	o.position = vec2(
		posX || 0,
		posY || 0
	);
	
	var centerOffset = vec2(
		w * 0.5,
		h * 0.5
	);

	o.worldToScreen = function(p)
	{
		return vec2(
			centerOffset.x + (p.x * o.scaleFactor) - o.position.x,
			centerOffset.y + (-p.y * o.scaleFactor) - o.position.y
		);
	};
};

function Physics2D(ctx, w, h, scale, centerOffsetx, centerOffsety)
{
	var o = {};

	// -------------------------------------------------------------------------------------------------------
	// Variables/Functions for performing the main functionality of this object
	// -------------------------------------------------------------------------------------------------------
	var rigidbodies = {};
	var forces = {};
	var collisions = [];
	var collisionMap = {};
	var solver = BasicDiscreteSolver();

	o.useGravity = true;
	o.forceOfGravity = vec2(0, -300);

	function ApplyForces(dt)
	{
		if(o.useGravity)
		{
			var gravity = o.forceOfGravity.multiply(dt);
			for(var i in rigidbodies)
			{
				if(rigidbodies[i].isKinematic)
					continue;

				rigidbodies[i].velocity = rigidbodies[i].velocity.add(gravity);
				console.log(rigidbodies[i].velocity);
			}
		}
		return;
		// Apply any forces in the list of forces to each rigidbody
		for(var i in forces)
		{
			if(forces[i].value.isZero())
				continue;

			var f = forces[i];

			for(var j in rigidbodies)
			{
				if(rigidbodies[j].isKinematic)
					continue;

				var rb = rigidbodies[j];

				if(f.isAcceleration)
					rb.velocity = rb.velocity.add(f.value.multiply(dt));
				else
					rb.force = rb.force.add(f.value.multiply(dt));
			}
		}
	};

	function CheckCollisions(dt)
	{
		collisions = [];
		collisionMap = {};

		for(var i in rigidbodies)
		{
			if(!rigidbodies[i].collider)
				continue;

			var a = rigidbodies[i];
			for(var j in rigidbodies)
			{
				if(i == j) 
					continue;

				if(!rigidbodies[j].collider)
					continue;

				var b = rigidbodies[j];

				if(a.isKinematic && b.isKinematic)
					continue;

				if(CheckCollisionPair(a.id, b.id))
					continue;

				MapPair(a.id, b.id);

				var c = solver.checkCollision(a, b, dt);
				if(c != null)
				{
					collisions.push(c);
					//console.log(c);
				}
			}
		}

		for(var i = 0; i < collisions.length; i++)
		{
			var c = collisions[i];
			c.a.OnPreCollision && c.a.OnPreCollision(c);
			c.b.OnPreCollision && c.b.OnPreCollision(c);
		}
		// Find all colliding rigidbodies.
		// Determine algorithm to used based on rigidbodies's colliders and possibly other factors
		// Only update a rigidbody's collider vertices before checking for collision
	};

	// Maps a pair of rigidbodies to each other indicating that they have collided 
	function MapPair(a, b)
	{
		if(!collisionMap[a])
			collisionMap[a] = {};

		collisionMap[a][b] = true;
		
		if(!collisionMap[b])
			collisionMap[b] = {};

		collisionMap[b][a] = true;
	};

	// Checks to see if a pair of rigidbodies has already been checked for collision
	function CheckCollisionPair(a, b)
	{
		return (collisionMap[a] && collisionMap[a][b]) || (collisionMap[b] && collisionMap[b][a]);
	};

	function ResolveCollisions(dt)
	{
		var gravity = o.forceOfGravity.multiply(o.useGravity ? dt * 2 : 0);

		// Resolve each collision we detected using the assigned solver
		for(var i = 0; i < collisions.length; i++)
		{
			var c = collisions[i];

			solver.resolveCollision(c, gravity);
			c.a.OnPostCollision && c.a.OnPostCollision(c);
			c.b.OnPostCollision && c.b.OnPostCollision(c);
		}
	};

	function CorrectPositions()
	{
		for(var i = 0; i < collisions.length; i++)
		{
			solver.correctPosition(collisions[i]);
		}
	};

	// Converts a point in cartesian space to werid flash-style screen space.
	o.toScreenCoords = function(v)
	{
		return vec2(v.x, h - v.y);
	};

	o.update = function(dt)
	{
		//var err = new Error();
		//console.log(err.stack);

		// Apply Forces
		ApplyForces(dt);

		// Check Collisions
		CheckCollisions();

		// Resolve Collisions
		ResolveCollisions(dt);

		// Update all rigidbodies to see where they end up
		// after the effects of the forces applied this step.
		for(var i in rigidbodies)
			rigidbodies[i].update(dt);

		// Account for any sinking into surfaces
		CorrectPositions();
	};

	o.addRigidbody = function(rigidbody)
	{
		rigidbodies[rigidbody.id] = rigidbody;
	};

	o.removeRigidbody = function(rigidbody)
	{
		rigidbodies[rigidbody.id] = null;
	};

	o.addUniversalForce = function(id, force, isAcceleration)
	{
		forces[id] = Force(force, isAcceleration);
	};

	o.removeUniversalForce = function(id)
	{
		forces[id] = null;
	};

	return o;
};


function Force(force, isAcceleration)
{
	var o = {};

	o.value = force;
	o.isAcceleration = isAcceleration;

	return o;
}


//---------------------------------------------------------------------------------------------------------------------------------------------
// 															ESSENTIAL STRUCTURES 
//---------------------------------------------------------------------------------------------------------------------------------------------


function GameObject()
{
	var o = {};

	o.transform = Transform();
	o.rigidbody = Rigidbody(o.transform);
	o.vertices = [];

	o.update = function(dt)
	{
		// Not sure what to do in here yet...
	};

	o.render = function(ctx)
	{
		// Debug
		o.rigidbody.collider.render(ctx);
	};

	o.addCircleCollider = function()
	{
		// Create a circle collider that fits the object's set of vertices
		// (max dist from center)
	};

	o.addBoxColider = function(axisAligned)
	{
		// Create a box collider that fits this object's set of vertices
		// (max point - min point)
	};

	return o;
};

function Transform()
{
	var o = {};

	o.position = vec2();
	o.rotation = 0;
	o.scale = vec2(1, 1);
	o.dirty = true;

	o.translate = function(v)
	{
		o.position.x += v.x;
		o.position.y += v.y;
	};

	o.rotate = function(angle)
	{
		o.rotation += angle;

		o.dirty = true;
	};

	o.setScale = function(x, y)
	{
		o.scale.x = x;
		o.scale.y = y;

		o.dirty = true;
	};

	return o;
};

function Rigidbody(transform)
{
	var o = {};

	o.id = (++g_rbId);
	o.collider;
	o.transform = transform;
	o.material = PhysicsMaterial();
	o.mass = 1;
	o.I = 1;
	o.invMass = 1;
	o.invI = 1;
	o.velocity = vec2();
	o.drag = 0;
	o.angularVelocity = 0;
	o.angularDrag = 0.05;
	o.force = vec2();
	o.torque = 0;
	o.isKinematic = false;
	o.constraints = {x: 0, y: 0};

	function updateCollider()
	{
		o.I = o.collider.calculateMomentOfInertia(o.mass);
		o.invI = 1.0/o.I;

		// Do something about rotating the collider's vertices according to the the transform
		o.collider.updateVertices();	
	};

	o.setCollider = function(collider)
	{
		o.collider = collider;
		o.collider.transform = o.transform;
		updateCollider();
	};

	o.resizeCollider = function()
	{
		o.collider.resize(arguments);
		updateCollider();
	};

	o.applyForce = function(force, isAcceleration)
	{
		if(isAcceleration)
			o.velocity = o.velocity.add(force);
		else
			o.force = o.force.add(force);
	};

	o.applyImpule = function(impulse, contactVector)
	{
		if(o.isKinematic)
			return;

		//console.log(impulse);
		o.velocity = o.velocity.add(impulse.multiply(o.invMass));

		if(!contactVector)
			return;

		o.angularVelocity += o.invI * contactVector.cross(impulse);
	};

	var prevMass = o.mass;
	o.update = function(dt)
	{
		// This helps save calculations at the cost of a little more memory
		if(o.mass != prevMass)
		{
			o.invMass = o.mass === 0 ? 0 : 1.0/o.mass;
			prevMass = o.mass;

			if(o.collider)
			{
				o.I = o.collider.calculateMomentOfInertia(o.mass);
				o.invI = 1.0/o.I;
			}
		}

		// If this rigidbody does not respond to physical reactions, no need to continue updating it
		if(!o.isKinematic)	
		{
			// Only update the position if there are forces acting on this object or if it has a velocity
			if(!(o.velocity.isZero() && o.force.isZero()))
			{
				var acceleration = o.force.isZero() ? o.force : o.force.multiply(o.invMass);
				o.velocity = acceleration.isZero() ? o.velocity : o.velocity.add(acceleration.multiply(dt));
				o.transform.translate(o.velocity.multiply(dt));
			}

			// Only update the rotation if this collider can rotate and there are forces acting on this object 
			// or if it has an angular velocity
			if(!(o.collider && o.collider.type === "AABB"))
			{
				//console.log("rotating!" + o.angularVelocity);
				//var angularAccel = o.torque === 0 ? 0 : o.torque * (o.invI | 0);
				//o.angularVelocity += o.angularAccel === 0 ? 0 : angularAccel * dt;
				o.transform.rotate(o.angularVelocity * dt);
			}

			o.velocity = o.velocity.multiply(1 - o.drag);
			o.angularVelocity *= (1 - o.angularDrag);
		}

		// Update this rigidbody's collider in case it has rotated or scaled
		if(o.collider)
		{
			o.collider.updateVertices();
		}

		// Clear all forces applied to this object for the next frame
		o.force.x = 0;
		o.force.y = 0;
		o.torque = 0;
	};
	
	return o;
};

var PhysicsMaterial = function()
{
	var o = {};

	o.elasticity = 0.2;
	o.dynamicFriction = 0.5;
	o.staticFriction = 0.3;

	return o;
};

//---------------------------------------------------------------------------------------------------------------------------------------------
// 															   	COLLIDERS
//---------------------------------------------------------------------------------------------------------------------------------------------

function CircleCollider(r)
{
	var o = {};

	o.type = "Circle";
	o.radius = r || 10;
	o.transform;

	o.resize = function()
	{
		var args = arguments[0];
		radius = args[0]; // 0 should be radius
	};

	o.calculateMomentOfInertia = function(mass)
	{
		return mass * o.radius * o.radius;
	};

	// Debug
	o.render = function(camera, color)
	{
		var ctx = camera.ctx;

		// Draw a circle
		ctx.strokeStyle = color || "#00FF00";
		ctx.beginPath();
		var screenPos = camera.worldToScreen(o.transform.position);
		ctx.arc(screenPos.x, screenPos.y, o.radius, 0, Math.PI * 2);
		ctx.stroke();

		ctx.strokeStyle = "#FF0000";
		ctx.beginPath();
		ctx.moveTo(screenPos.x, screenPos.y);
		// Rotations are reversed because of camera coordinates
		ctx.lineTo(screenPos.x + Math.cos(-o.transform.rotation) * o.radius, screenPos.y + Math.sin(-o.transform.rotation) * o.radius);
		ctx.stroke();
	};

	o.updateVertices = function()
	{
		// Do nothing. This is a circle.
	};

	return o;
};

function AABBCollider(w, h)
{
	var o = {};

	o.type = "AABB";
	o.halfBounds = vec2(w * 0.5 || 10, h * 0.5 || 10);
	o.vertices;
	o.transform;

	o.resize = function()
	{
		var args = arguments[0];
		o.halfBounds.x = args[0] * 0.5; // 0 should be width
		o.halfBounds.y = args[1] * 0.5; // 1 should be height

		// Update this collider's vertices to reflect the new size
		o.transform.dirty = true;
		o.updateVertices();
	};

	o.calculateMomentOfInertia = function(mass)
	{
		var width = o.halfBounds.x * 2;
		var height = o.halfBounds.y * 2;
		return (mass * (width * width + height * height))/12;
	};

	o.render = function(camera, color)
	{
		var ctx = camera.ctx;

		ctx.strokeStyle = color || "#00FF00";
		var screenPos = camera.worldToScreen(o.transform.position);
		ctx.strokeRect(screenPos.x - o.halfBounds.x, screenPos.y - o.halfBounds.y, o.halfBounds.x * 2, o.halfBounds.y * 2);
	};

	o.updateVertices = function()
	{
		if(!o.transform || !o.transform.dirty)
			return;

		// Let's do our best to minimize unnecessarily allocating new vec2's during this process.
		if(!o.vertices)
		{
			o.vertices = [vec2(), vec2(), vec2(), vec2()];
		}

		o.vertices[0].set(-o.halfBounds.x, o.halfBounds.y); // Top Left
		o.vertices[1].set(o.halfBounds.x, o.halfBounds.y); // Top Right
		o.vertices[2].set(o.halfBounds.x, -o.halfBounds.y); // Bottom Right
		o.vertices[3].set(-o.halfBounds.x, -o.halfBounds.y); // Bottom Left

		o.transform.dirty = false;
	};

	return o;
}

function OBBCollider(w, h)
{
	// An OBB is just an AABB that can rotate, so we can just use an AABB as the base and override some of the functionality.
	var o = AABBCollider(w, h); 

	o.type = "OBB";
	o.worldAxes = [vec2(1, 0), vec2(0, 1)]; // World space axes. These change with the OBB's orientation and are used for projection during SAT checks.
	o.localAxes = [vec2(1, 0), vec2(0, 1)]; // We need local axes as well, so we always know what to start with when calculating the world axes after rotation.

	o.updateVertices = function()
	{
		// If there is nothing to update, return.
		if(!o.transform || !o.transform.dirty)
			return;

		// Let's do our best to minimize unnecessarily allocating new vec2's during this process.
		if(!o.vertices)
		{
			o.vertices = [vec2(), vec2(), vec2(), vec2()];
		}

		// Update Axes
		o.worldAxes[0] = o.localAxes[0].rotate(o.transform.rotation);
		o.worldAxes[1] = o.worldAxes[0].perp(true);

		// Update vertices
		var xoffset = o.worldAxes[0].multiply(o.halfBounds.x);
		var yoffset = o.worldAxes[1].multiply(o.halfBounds.y);

		o.vertices[0].set(-xoffset.x + yoffset.x, -xoffset.y + yoffset.y);
		o.vertices[1].set(xoffset.x + yoffset.x, xoffset.y + yoffset.y);
		o.vertices[2].set(xoffset.x - yoffset.x, xoffset.y - yoffset.y);
		o.vertices[3].set(-xoffset.x - yoffset.x, -xoffset.y - yoffset.y);

		o.transform.dirty = false;
	};

	o.render = function(camera, color)
	{
		var ctx = camera.ctx;

		// Draw a rotated rectangle
		ctx.strokeStyle = color || "#00FF00";
		var screenPos = camera.worldToScreen(o.transform.position);
		ctx.save();
		ctx.translate(screenPos.x, screenPos.y);
		ctx.rotate(-o.transform.rotation);
		ctx.strokeRect(-o.halfBounds.x, -o.halfBounds.y, o.halfBounds.x << 1, o.halfBounds.y << 1);
		ctx.restore();

		for(var i = 0; i < o.worldAxes.length; i++)
		{
			ctx.strokeStyle = i % 2 == 0 ? "#FF0000" : "#0000FF";
			var endPoint = camera.worldToScreen(o.transform.position.add(o.worldAxes[i].multiply(30)));
			ctx.beginPath();
			ctx.moveTo(screenPos.x, screenPos.y);
			ctx.lineTo(endPoint.x, endPoint.y);
			ctx.stroke();
		}
	};

	return o;
}

//---------------------------------------------------------------------------------------------------------------------------------------------
// 													COLLISION DETECTION / RESOLUTION
//---------------------------------------------------------------------------------------------------------------------------------------------

var CollisionType = {
	Circle: 		0,				// Two Circles
	AABB: 			1,				// Two AABB's
	OBB: 			2,				// Two OBB's
	AABBCircle: 	3,
	CircleOBB: 		4,
	AABBOBB: 		5
};

// This is a convenience function to be used when we are generating collision data, and know that either a or b is of the 
// specified type, but not which. It returns an object containing the two colliders with the collider matching the 
// specified type as a, and the other as b.
function OrderByType(type, a, b)
{
	return a.collider.type === type ? {a: a, b: b} : {a: b, b: a};
}

function CollisionData(a, b)
{
	// Same-Type collisions first
	if(a.collider.type === b.collider.type)
	{
		if(a.collider.type === "Circle")
		{
			return {type: CollisionType.Circle, a: a, b: b};
		}

		if(a.collider.type === "AABB")
		{
			return {type: CollisionType.AABB, a: a, b: b};
		}

		if(a.collider.type === "OBB")
		{
			return {type: CollisionType.OBB, a: a, b: b};
		}
	}

	// AABB -> ? 
	if(a.collider.type === "AABB" || b.collider.type === "AABB")
	{
		var ordered = OrderByType("AABB", a, b);
		if(ordered.b.collider.type === "Circle")
		{
			// a is the circle for my own personal ease of visualization
			return {type: CollisionType.AABBCircle, a: ordered.b, b: ordered.a};
		}

		if(ordered.b.collider.type === "OBB")
		{
			return {type: CollisionType.AABBOBB, a: ordered.a, b: ordered.b};
		}
	}

	// Circle -> ?
	if(a.collider.type === "Circle" || b.collider.type === "Circle")
	{
		var ordered = OrderByType("Circle", a, b);
		if(ordered.b.collider.type === "OBB")
		{
			return {type: CollisionType.CircleOBB, a: ordered.a, b: ordered.b};
		}
	}

	// Unrecognized collision type
	return null;
};

function BasicDiscreteSolver()
{
	var o = {};

	//-------------------------------------------------------------------------------------------------------------------------
	// AABB CHECKS
	//-------------------------------------------------------------------------------------------------------------------------

	// AABB -> AABB
	function CheckAABB(a, b, collision)
	{
		var btoa = a.transform.position.subtract(b.transform.position);
		var dxdy = vec2(Math.abs(btoa.x), Math.abs(btoa.y));
		var minDist = a.collider.halfBounds.add(b.collider.halfBounds); 
		var pen = minDist.subtract(dxdy);

		if(pen.x > 0 && pen.y > 0)
		{
			// Hit!
			collision.penetration = Math.min(pen.x, pen.y);
			collision.btoa = btoa;
			if(pen.x < pen.y)
			{
				collision.normal = btoa.x < 0 ? vec2(-1, 0) : vec2(1, 0);
			}
			else
			{
				collision.normal = btoa.y < 0 ? vec2(0, -1) : vec2(0, 1);
			}

			return collision;
		}

		return null;
	};


	//-------------------------------------------------------------------------------------------------------------------------
	// CIRCLE CHECKS
	//-------------------------------------------------------------------------------------------------------------------------

	function CircleBoxCollision(circlePos, boxPos, circleCollider, boxCollider, collision)
	{
		var halfBounds = boxCollider.halfBounds;
		var apos = circlePos;
		var bpos = boxPos;
		var btoa = apos.subtract(bpos);
		var closestPoint = vec2(clamp(btoa.x, -halfBounds.x, halfBounds.x),
								clamp(btoa.y, -halfBounds.y, halfBounds.y));


		// If the closest point is inside the AABB, we need to calculate the penetration by finding the minimum
		// distance to one of the AABB's sides
		if(closestPoint.x == btoa.x && closestPoint.y == btoa.y)
		{	
			// Hit!
			var pen = vec2(halfBounds.x - Math.abs(btoa.x), halfBounds.y - Math.abs(btoa.y));
			collision.penetration = Math.min(pen.x, pen.y) + circleCollider.radius;
			if(pen.x < pen.y)
			{
				collision.normal = btoa.x < 0 ? vec2(-1, 0) : vec2(1, 0);
			}
			else
			{
				collision.normal = btoa.y < 0 ? vec2(0, -1) : vec2(0, 1);
			}

			// Let's leave a flag that can be used to easily distinguish whether the circle's center was
			// inside the AABB. This can be used to help us resolve the collision more efficiently.
			collision.circleInterior = true;
		}
		// If the closest point lies on one of the AABB's edges, we can find if we are in collision by
		// seeing if the distance from the circle's center to that point is less than the radius. If so,
		// we can use the differnce as the penetration and the vector from the closest point to the 
		// circle's center as the normal.
		{
			closestPoint = closestPoint.add(bpos);
			var p2c = apos.subtract(closestPoint);
			var sqrdst = p2c.sqrmag();
			if(sqrdst < circleCollider.radius * circleCollider.radius)
			{
				// Hit!
				collision.penetration = circleCollider.radius - Math.sqrt(sqrdst);
				collision.normal = p2c.normalize();
			}
			else
			{
				return null;
			}
		}

		collision.btoa = btoa;
		return collision;
	};

	// Circle -> Circle
	function CheckCircle(a, b, collision)
	{
		var btoa = a.transform.position.subtract(b.transform.position);
		var dist = btoa.sqrmag();
		var minDist = a.collider.radius + b.collider.radius;
		if(dist < (minDist * minDist))
		{
			collision.penetration = minDist - Math.sqrt(dist);
			collision.btoa = btoa;
			collision.normal = btoa.normalize();

			// we will need to adjust this by the amount we move b when we separate the objects later.
			collision.contactPoints = [a.transform.position.add(collision.normal.multiply(-a.collider.radius))];
			return collision;
		}

		return null;
	};

	// Circle -> AABB
	// a = circle
	// b = AABB
	function CheckCircleAABB(a, b, collision)
	{ 
		var result = CircleBoxCollision(a.transform.position, b.transform.position, a.collider, b.collider, collision);

		if(result)
		{
			// we will need to adjust this by the amount we move b when we separate the objects later.
			result.contactPoint = [a.transform.position.add(result.normal.multiply(-a.collider.radius))];
		}

		return result;
	};

	// Circle -> OBB
	// a = circle
	// b = OBB
	function CheckCircleOBB(a, b, collision)
	{
		var btoa = a.transform.position.subtract(b.transform.position);

		// Before performing the usual check, let's transform the circle into the OBB's coordinate space, that way we can
		// pretend the OBB is at the origin with no rotation and use the same technique we use for testing Circles against AABBs.
		var circlePos = btoa.rotate(-b.transform.rotation);
		var result = CircleBoxCollision(circlePos, vec2(), a.collider, b.collider, collision);

		if(result)
		{
			result.normal = result.normal.rotate(b.transform.rotation);
			result.btoa = btoa;

			// we will need to adjust this by the amount we move b when we separate the objects later.
			result.contactPoint = [a.transform.position.add(result.normal.multiply(-a.collider.radius))];
		}

		return result;
	};

	//-------------------------------------------------------------------------------------------------------------------------
	// SEPARATING AXIS CHECKS
	//-------------------------------------------------------------------------------------------------------------------------

	var multiPointThreshold = 5.1;

	// Helper function to find the min/max vertex projections along the x and y axes for Rigidbody rb.
	function GetMinMaxPointsXY(rb)
	{
		if(!rb.collider || !rb.collider.vertices)
			return;

		var result = {
			minX: 	null, 
			maxX: 	null, 
			minY: 	null, 
			maxY: 	null,
			minPX: 	null,
			maxPX: 	null,
			minPY: 	null,
			minPX: 	null
		};

		for(var i = 0; i < rb.collider.vertices.length; i++)
		{
			var v = rb.collider.vertices[i].add(rb.transform.position);

			if(result.minX === null || v.x <= result.minX)
			{
				// if(result.minX && Math.abs(result.minX - v.x) < multiPointThreshold)
				// 	console.log("Hit!!", result.minX, v.x);
				// else
				// 	console.log("Miss...", result.minX, v.x, result.minPX);
				//console.log(result.minX - v.x);
				//(result.minX && Math.abs(result.minX - v.x) < multiPointThreshold) ? result.minPX.push(v) : result.minPX = [v];
				result.minPX = [v];
				result.minX = v.x;
			}
			if(result.maxX === null || v.x >= result.maxX)
			{
				// if(result.maxX &&  Math.abs(v.x - result.maxX) < multiPointThreshold)
				// 	console.log("Hit!", result.maxX, v.x);
				// else
				// 	console.log("Miss...", result.maxX, v.x, result.maxPX);

				//console.log(v.x - result.maxX);
				//(result.maxX &&  Math.abs(v.x - result.maxX) < multiPointThreshold) ? result.maxPX.push(v) : result.maxPX = [v];
				result.maxPX = [v];
				result.maxX = v.x;
			}
			if(result.minY === null || v.y <= result.minY)
			{
				// if(result.minY && Math.abs(result.minY - v.y) < multiPointThreshold)
				// 	console.log("Hit!", result.minY, v.y);
				// else
				// 	console.log("Miss...", result.minY, v.y, result.minPY);
				//console.log(result.minY - v.y);
				//(result.minY && Math.abs(result.minY - v.y) < multiPointThreshold) ? result.minPY.push(v) : result.minPY = [v];
				result.minPY = [v];
				result.minY = v.y;
			}
			if(result.maxY === null || v.y >= result.maxY)
			{
				// if(result.maxY && Math.abs(v.y - result.maxY) < multiPointThreshold)
				// 	console.log("Hit!", result.maxY, v.y);
				// else
				// 	console.log("Miss...", result.maxY, v.y, result.maxPY);
				//console.log(v.y - result.maxY);
				//(result.maxY && Math.abs(v.y - result.maxY) < multiPointThreshold) ? result.maxPY.push(v) : result.maxPY = [v];
				result.maxPY = [v];
				result.maxY = v.y;
			}
		}

		//console.log(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>", result);
		return result;
	};

	// Helper function to find the min/max vertex projections along the given axis relative to the
	// specified origin.
	function ProjectOntoAxis(rb, origin, axis)
	{
		if(!rb.collider || !rb.collider.vertices)
			return;

		var result = {
			min: 	null, 
			max: 	null,
			minP: 	null,
			maxP: 	null
		};

		for(var i = 0; i < rb.collider.vertices.length; i++)
		{
			var p = origin ? rb.collider.vertices[i].add(rb.transform.position).subtract(origin) : rb.collider.vertices[i];
			var dist = p.dot(axis);

			if(result.min === null || dist <= result.min)
			{
				// if(result.min && Math.abs(result.min - dist) < multiPointThreshold)
				// 	console.log("Hit!", result.min, dist);
				// else
				// 	console.log("Miss...", result.min, dist, result.minP);

				var p = rb.collider.vertices[i].add(rb.transform.position);
				//(result.min && Math.abs(result.min - dist) < multiPointThreshold) ? result.minP.push(p) : result.minP =  [p];
				result.minP = [p];
				result.min = dist;
			}

			if(result.max === null || dist >= result.max)
			{
				// if(result.max && Math.abs(result.max - dist) < multiPointThreshold)
				// 	console.log("Hit!", result.max, dist);
				// else
				// 	console.log("Miss...", result.max, dist, result.maxP);

				var p = rb.collider.vertices[i].add(rb.transform.position);
				//(result.max && Math.abs(dist - result.max) < multiPointThreshold) ? result.maxP.push(p) : result.maxP = [p];
				result.maxP = [p];
				result.max = dist;
			}
		}

		//console.log(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>", result);
		return result;
	};

	// Projects the vertices of OBB a and AABB b onto the axes of b and checks for overlap.
	// Significantly faster than ProjectAxes because it takes advantage of the axis-aligned 
	// nature of AABB's to avoid using actual dot products and vector calculations.
	// returns the min penetration and the normal of the corresponding surface on b.
	function ProjectAxesAABB(a, b)
	{
		var projA = GetMinMaxPointsXY(a);
		var projB = GetMinMaxPointsXY(b);

		var result = {pen: null, normal: null};
		
		// Check for overlap in the x-axis
		if(projB.minX > projA.maxX || projA.minX > projB.maxX)
		{
			// No overlap? No collision!
			return null;
		}
		else
		{
			var baoverlap = projB.maxX - projA.minX;
			var aboverlap = projA.maxX - projB.minX;
			if(baoverlap < aboverlap)
			{
				result.pen = baoverlap;
				result.normal = vec2(1, 0);
				result.contactPoint = projA.minPX;
			}
			else
			{
				// use amaxx
				result.pen = aboverlap;
				result.normal = vec2(-1, 0);
				result.contactPoint = projA.maxPX;
			}
		}

		// Check for overlap in the y-axis
		if(projB.minY > projA.maxY || projA.minY > projB.maxY)
		{
			// No overlap? No collision!
			return null;
		}
		else
		{
			var baoverlap = projB.maxY - projA.minY;
			var aboverlap = projA.maxY - projB.minY;
			if(baoverlap < aboverlap)
			{
				var pen = baoverlap;
				if(pen < result.pen)
				{
					result.pen = pen;
					result.normal = vec2(0, 1);
					result.contactPoint = projA.minPY;
				}
			}
			else
			{
				var pen = aboverlap;
				if(pen < result.pen)
				{
					result.pen = pen;
					result.normal = vec2(0, -1);
					result.contactPoint = projA.maxPY;
				}
			}
		}

		return result;
	};

	// Projects the vertices of OBBs a and b onto b's rotated axes and checkes for overlap.
	// Returns the min penetration and the normal of the corresponding surface on b.
	function ProjectAxes(a, b)
	{
		var result = {pen: null, normal: null};

		for(var i = 0; i < b.collider.worldAxes.length; i++)
		{
			var axis = b.collider.worldAxes[i];
			var projA = ProjectOntoAxis(a, b.transform.position, axis);
			var projB = ProjectOntoAxis(b, null, axis);

			if(projB.min > projA.max || projA.min > projB.max)
			{
				// No overlap? No collision!
				return null;
			}
			else
			{
				var baoverlap = projB.max - projA.min;
				var aboverlap = projA.max - projB.min;
				if(baoverlap < aboverlap)
				{
					// Use amin vertex as contact point
					var pen = baoverlap;
					if(result.pen === null || pen < result.pen)
					{
						result.pen = pen;
						result.normal = axis;
						result.contactPoint = projA.minP;
					}
				}
				else
				{
					// Use amax vertex as contact point
					var pen = aboverlap;
					if(result.pen === null || pen < result.pen)
					{
						result.pen = pen;
						result.normal = axis.multiply(-1);
						result.contactPoint = projA.maxP;
					}
				}
			}
		}

		return result;
	};

	// Checks to find a separating axis between two OBBs or an AABB and an OBB.
	// (Although, this check is designed in a way that it could be used for any pair of polygons).
	function SeparatingAxisCheck(a, b, collision)
	{
		var result = null;

		// Let's start by projecting b onto a's axes.

		// If one collider is an AABB, we can take a shortcut.
		if(a.collider.type === "AABB")
		{
			result = ProjectAxesAABB(b, a);
		}
		// Otherwise, it looks like both objects are OBB's. We'll need to do things
		// the hard way.
		else
		{
			result = ProjectAxes(b, a);
		}

		// If we found a separating axis, we can safely conclude that the objects
		// are not colliding.
		if(result === null)
			return;

		collision.penetration = result.pen;
		collision.normal = result.normal.multiply(-1); // The normal needs to be flipped because the resolution algorithm assumes the normal points from b's surface to a.
		collision.contactPoint = result.contactPoint;

		// Perform a second check, this time, projecting a onto b's axes
		result = ProjectAxes(a, b);

		// If the second check turned up a separating axis, there is no collision!
		if(result === null)
			return;

		if(result.pen < collision.penetration)
		{
			collision.penetration = result.pen;
			collision.normal = result.normal;
			collision.contactPoint = result.contactPoint;
		}

		collision.btoa = a.transform.position.subtract(b.transform.position);

		return collision;
	};

	//-------------------------------------------------------------------------------------------------------------------------
	// GENERAL COLLISION CHECK FUNCTIONS
	//-------------------------------------------------------------------------------------------------------------------------

	function EstimateEarliestCollision(a, b, checkFunc, collision, dt, iterations)
	{
		if(!collision)
			return null;

		dt = dt || 1/30;
		iterations = iterations || 4;
		collision.apos = a.transform.position;
		collision.bpos = b.transform.position;

		var astart = a.transform.position.subtract(a.velocity.multiply(dt));
		var bstart = b.transform.position.subtract(b.velocity.multiply(dt));
		var dA = a.transform.position.subtract(astart);
		var dB = b.transform.position.subtract(bstart);

		var t = 0.5;
		var step = 0.25;
		var earliest = collision;
		for(var i = 0; i < iterations; i++)
		{
			a.transform.position = astart.add(dA.multiply(t));
			b.transform.position = bstart.add(dB.multiply(t));
			var col = checkFunc(a, b, collision);

			if(col)
			{
				earliest = col;
				col.apos = vec2(a.transform.position.x, a.transform.position.y);
				col.bpos = vec2(b.transform.position.x, b.transform.position.y);
				t -= step;
			}
			else
			{
				t += step;
			}

			step = step * 0.5;
		}

		a.transform.position = earliest.apos;
		b.transform.position = earliest.bpos;
		return earliest;
	};

	o.checkCollision = function(a, b, dt)
	{
		var collision = CollisionData(a, b);
		var checkFunc;

		// Circle -> Circle
		if(collision.type === CollisionType.Circle)
		{
			collision = CheckCircle(collision.a, collision.b, collision);
			checkFunc = CheckCircle;
		}

		// AABB -> AABB
		else if(collision.type === CollisionType.AABB)
		{
			collision = CheckAABB(collision.a, collision.b, collision);
			checkFunc = CheckAABB;
		}

		// Circle -> AABB
		// a = circle
		// b = AABB
		else if(collision.type === CollisionType.AABBCircle)
		{
			collision = CheckCircleAABB(collision.a, collision.b, collision);
			checkFunc = CheckCircleAABB;
		}

		// Circle -> OBB
		// a = circle
		// b = OBB
		else if(collision.type === CollisionType.CircleOBB)
		{
			collision = CheckCircleOBB(collision.a, collision.b, collision);
			checkFunc = CheckCircleOBB;
		}

		// ??? -> OBB
		// a = AABB/OBB
		// b = OBB
		else if(collision.type === CollisionType.AABBOBB || collision.type === CollisionType.OBB)
		{
			collision = SeparatingAxisCheck(collision.a, collision.b, collision);
			checkFunc = SeparatingAxisCheck;
		}


		if(!collision)
			return;

		// This check is used to determine if the objects have penetrated each other enough to have passed through each other's centers.
		// In these situations, it is necessary to resolve the collision using a binary search for the earliest collision point. Otherwise,
		// it is more efficient to avoid the extra checks. In addition, since the binary search can cause problems in situations where the object
		// moved very little along the axis of collision but a lot along a nearly perpendicular axis, this check is beneficial in preventing the
		// binary search from happening in those scenarios.
		var startPosA = collision.a.transform.position.subtract(collision.a.velocity.multiply(dt));
		var startPosB = collision.b.transform.position.subtract(collision.b.velocity.multiply(dt));
		var startBtoA = startPosA.subtract(startPosB);
		if(collision.circleInterior || startBtoA.dot(collision.btoa) <= 0)
		{
			return EstimateEarliestCollision(collision.a, collision.b, checkFunc, collision, dt, 4);
		}

		return collision;
	};

	//-------------------------------------------------------------------------------------------------------------------------
	// RESOLUTION FUNCTIONS
	//-------------------------------------------------------------------------------------------------------------------------

	o.resolveCollision = function(collision, gravity)
	{
		console.log("Resolving Collisions!");
		if(collision.type === CollisionType.AABB)
			o.resolveCollisionWithoutTorque(collision, gravity)
		else
			o.resolveCollisionTorque(collision, gravity);
	}

	o.resolveCollisionTorque = function(collision, gravity)
	{
		var a = collision.a;
		var b = collision.b;
		var n = collision.normal;
		var d = collision.penetration;
		
		// -----------------------------------------------------
		// Move the objects out of collision with each other
		// -----------------------------------------------------
		var aveln = Math.abs(a.velocity.dot(n));
		var bveln = Math.abs(b.velocity.dot(n));
		var ares = 0;
		var bres = 0;

		if(aveln > 0 && bveln > 0 && !a.isKinematic && !b.isKinematic)
		{
			//console.log("This ONe!");
			//console.log(aveln);
			//console.log(bveln);
			//console.log(d);
			var tveln = aveln + bveln;
			//console.log(tveln);
			ares = (aveln/tveln) * d;
			bres = (bveln/tveln) * d;
			//collision.contactPoint = collision.contactPoint.add(n.multiply(ares));
			a.transform.translate(n.multiply(ares));
			b.transform.translate(n.multiply(-bres));
			//console.log(ares);
			//console.log(bres);
		}
		else if(aveln > 0 || b.isKinematic)
		{
			ares = d;
			a.transform.translate(n.multiply(ares));
			//collision.contactPoint = collision.contactPoint.add(n.multiply(ares));
		}
		else if(bveln > 0 || a.isKinematic)
		{
			bres = -d;
			b.transform.translate(n.multiply(bres));
			//collision.contactPoint = collision.contactPoint.add(n.multiply(bres));
		}
		else
		{
			ares = d * 0.5;
			bres = -ares;
			//collision.contactPoint = collision.contactPoint.add(n.multiply(ares));
			a.transform.translate(n.multiply(ares));
			b.transform.translate(n.multiply(bres));
		}

		var length = collision.contactPoint.length;
		for(var i = 0; i < length; i++)
		{
			var contactPoint = collision.contactPoint[i];
			window.contactPoint = contactPoint;

			var ra = contactPoint.subtract(collision.a.transform.position);
			var rb = contactPoint.subtract(collision.b.transform.position);
			
			// Apply restitution impulse
			//var relativeVelocity = a.velocity.subtract(b.velocity);
			var relativeVelocity = a.velocity.add(crossSV(a.angularVelocity, ra)).subtract(b.velocity).subtract(crossSV(b.angularVelocity, rb));
			var contactVelocity = relativeVelocity.dot(n);

			if(contactVelocity > 0)
				return;

			var raCrossN = Math.pow(ra.cross(n), 2) * a.invI;
			var rbCrossN = Math.pow(rb.cross(n), 2) * b.invI;
			var invMassSum = a.isKinematic ? (b.invMass + rbCrossN) :
							 b.isKinematic ? (a.invMass + raCrossN) :
							 a.invMass + raCrossN + b.invMass + rbCrossN;

			var e = relativeVelocity.sqrmag() <= gravity.sqrmag() + 1 ? 0 : 1 + Math.min(a.material.elasticity, b.material.elasticity);
			console.log(e, relativeVelocity.sqrmag(), gravity.sqrmag());
			var I = -(1 * contactVelocity)/invMassSum;
			I /= length;
			var impulse = n.multiply(I);

			a.applyImpule(impulse, ra);
			b.applyImpule(impulse.multiply(-1), rb);

			// Apply friction impulse
			var staticFriction = Math.min(a.material.staticFriction, b.material.staticFriction);
			var dynamicFriction = Math.min(a.material.dynamicFriction, b.material.dynamicFriction);

			//relativeVelocity = a.velocity.subtract(b.velocity);
			relativeVelocity = a.velocity.add(crossSV(a.angularVelocity, ra)).subtract(b.velocity).subtract(crossSV(b.angularVelocity, rb));
			//console.log(relativeVelocity);
			//console.log("------------------------------------------------------");
			//return;
			var z =  relativeVelocity.subtract(n.multiply(relativeVelocity.dot(n)));
			var t = relativeVelocity.subtract(n.multiply(relativeVelocity.dot(n))).normalize();
			var IT = -relativeVelocity.dot(t)/invMassSum;
			IT /= length;
			//console.log(IT, a.angularVelocity, I, staticFriction, a.velocity, t.multiply(IT * a.invMass));
			//if(Math.abs(IT) <= 0.0001)
				//return;

			var tangentImpulse = Math.abs(IT) < I * staticFriction ? tangentImpulse = t.multiply(IT) : 
																 tangentImpulse = t.multiply(-I * dynamicFriction);

			//console.log(z, relativeVelocity, t, IT);
			//console.log("---------------------------------");
			a.applyImpule(tangentImpulse, ra);
			b.applyImpule(tangentImpulse.multiply(-1), rb);
		}
		//console.log(tangentImpulse, ra, ra.cross(tangentImpulse), "Cats");
	};

	o.resolveCollisionWithoutTorque = function(collision, gravity)
	{
		var a = collision.a;
		var b = collision.b;
		var n = collision.normal;
		var d = collision.penetration;
		
		// -----------------------------------------------------
		// Move the objects out of collision with each other
		// -----------------------------------------------------
		var aveln = Math.abs(a.velocity.dot(n));
		var bveln = Math.abs(b.velocity.dot(n));
		var ares = 0;
		var bres = 0;

		if(aveln > 0 && bveln > 0 && !a.isKinematic && !b.isKinematic)
		{
			//console.log("This ONe!");
			//console.log(aveln);
			//console.log(bveln);
			//console.log(d);
			var tveln = aveln + bveln;
			//console.log(tveln);
			ares = (aveln/tveln) * d;
			bres = (bveln/tveln) * d;
			//collision.contactPoint = collision.contactPoint.add(n.multiply(ares));
			a.transform.translate(n.multiply(ares));
			b.transform.translate(n.multiply(-bres));
			//console.log(ares);
			//console.log(bres);
		}
		else if(aveln > 0 || b.isKinematic)
		{
			ares = d;
			a.transform.translate(n.multiply(ares));
			//collision.contactPoint = collision.contactPoint.add(n.multiply(ares));
		}
		else if(bveln > 0 || a.isKinematic)
		{
			bres = -d;
			b.transform.translate(n.multiply(bres));
			//collision.contactPoint = collision.contactPoint.add(n.multiply(bres));
		}
		else
		{
			ares = d * 0.5;
			bres = -ares;
			//collision.contactPoint = collision.contactPoint.add(n.multiply(ares));
			a.transform.translate(n.multiply(ares));
			b.transform.translate(n.multiply(bres));
		}
		
		// Apply restitution impulse
		//var relativeVelocity = a.velocity.subtract(b.velocity);
		var relativeVelocity = a.velocity.subtract(b.velocity);
		var contactVelocity = relativeVelocity.dot(n);

		if(contactVelocity > 0)
			return;

		
		var invMassSum = a.isKinematic ? b.invMass :
						 b.isKinematic ? a.invMass :
						 a.invMass + b.invMass;


		var e = 1 + Math.min(a.material.elasticity, b.material.elasticity);
		var I = -(e * contactVelocity)/invMassSum;
		var impulse = n.multiply(I);

		a.applyImpule(impulse);
		b.applyImpule(impulse.multiply(-1));

		// Apply friction impulse
		var staticFriction = Math.min(a.material.staticFriction, b.material.staticFriction);
		var dynamicFriction = Math.min(a.material.dynamicFriction, b.material.dynamicFriction);

		//relativeVelocity = a.velocity.subtract(b.velocity);
		var relativeVelocity = a.velocity.subtract(b.velocity);
		//console.log(relativeVelocity);
		//console.log("------------------------------------------------------");
		//return;
		var z =  relativeVelocity.subtract(n.multiply(relativeVelocity.dot(n)));
		var t = relativeVelocity.subtract(n.multiply(relativeVelocity.dot(n))).normalize();
		var IT = -relativeVelocity.dot(t)/invMassSum;
		//console.log(IT, a.angularVelocity, I, staticFriction, a.velocity, t.multiply(IT * a.invMass));
		//if(Math.abs(IT) <= 0.0001)
			//return;

		var tangentImpulse = Math.abs(IT) < I * staticFriction ? tangentImpulse = t.multiply(IT) : 
																 tangentImpulse = t.multiply(-I * dynamicFriction);

		//console.log(z, relativeVelocity, t, IT);
		//console.log("---------------------------------");
		a.applyImpule(tangentImpulse);
		b.applyImpule(tangentImpulse.multiply(-1));
	};

	o.correctPosition = function(collision)
	{
		return;

		var a = collision.a;
		var b = collision.b;
		var slop = 0.05;
		var percent = 0.4;
		var massSum =  a.isKinematic ? b.invMass :
					   b.isKinematic ? a.invMass :
					   a.invMass + b.invMass;

		var correction = collision.normal.multiply((Math.max(collision.penetration - slop, 0.0)/massSum) * percent);
		
		if(!a.isKinematic)
			a.transform.translate(correction);
		if(!b.isKinematic)
			b.transform.translate(correction.multiply(-1));
	}

	// o.resolveCollision = function(collision)
	// {
	// 	var a = collision.a;
	// 	var b = collision.b;
	// 	var n = collision.normal;
	// 	var d = collision.penetration;
		
	// 	// -----------------------------------------------------
	// 	// Move the objects out of collision with each other
	// 	// -----------------------------------------------------
	// 	var aveln = Math.abs(a.velocity.dot(n));
	// 	var bveln = Math.abs(b.velocity.dot(n));
	// 	var ares = 0;
	// 	var bres = 0;

	// 	if(aveln > 0 && bveln > 0 && !a.isKinematic && !b.isKinematic)
	// 	{
	// 		//console.log("This ONe!");
	// 		//console.log(aveln);
	// 		//console.log(bveln);
	// 		//console.log(d);
	// 		var tveln = aveln + bveln;
	// 		//console.log(tveln);
	// 		ares = (aveln/tveln) * d;
	// 		bres = (bveln/tveln) * d;
	// 		a.transform.translate(n.multiply(ares));
	// 		b.transform.translate(n.multiply(-bres));
	// 		//console.log(ares);
	// 		//console.log(bres);
	// 	}
	// 	else if(aveln > 0 || b.isKinematic)
	// 	{
	// 		ares = d;
	// 		a.transform.translate(n.multiply(ares));
	// 	}
	// 	else if(bveln > 0 || a.isKinematic)
	// 	{
	// 		bres = -d;
	// 		b.transform.translate(n.multiply(bres));
	// 	}
	// 	else
	// 	{
	// 		ares = d * 0.5;
	// 		bres = -ares;
	// 		a.transform.translate(n.multiply(ares));
	// 		b.transform.translate(n.multiply(bres));
	// 	}
		
	// 	// -----------------------------------------------------
	// 	// Appyly friction forces
	// 	// -----------------------------------------------------


	// 	// Changes to make:
	// 	// ------------------------------------------------------
	// 	// - Calculate torque along with I.
	// 	// - This can probably be done in a way similar to I? Add together the torques at the specified points then project it onto n, then divide by the combined moment of inertia?
	// 	// - 
	// 	// -----------------------------------------------------
	// 	// Apply appropriate repulsion forces to each object
	// 	// -----------------------------------------------------
	// 	var rvel = a.velocity.subtract(b.velocity);

	// 	//var e = (a.material.elasticity + b.material.elasticity) * 0.5;
	// 	var e = 1 + Math.min(a.material.elasticity, b.material.elasticity);
	// 	var I = rvel.projectOnto(n);

	// 	var totalMass = a.isKinematic ? b.invMass :
	// 					b.isKinematic ? a.invMass :
	// 					a.invMass + b.invMass;

	// 	I = I.divide(totalMass);
	// 	a.velocity = a.isKinematic ? vec2() : a.velocity.add(I.multiply(e * -a.invMass));
	// 	b.velocity = b.isKinematic ? vec2() : b.velocity.add(I.multiply(e * b.invMass));

	// 	/*
	// 	console.log(a.collider.type);
	// 	console.log(b.collider.type);
	// 	console.log(a.velocity);
	// 	console.log(b.velocity);
	// 	*/
	// };

	return o;
};


//---------------------------------------------------------------------------------------------------------------------------------------------
// 																UTILITIES
//---------------------------------------------------------------------------------------------------------------------------------------------


function vec2(x, y)
{
	var o = {};

	o.x = x || 0;
	o.y = y || 0;

	o.set = function(x, y)
	{
		o.x = x || 0;
		o.y = y || 0;
	};

	o.add = function(v)
	{
		return vec2(o.x + v.x, o.y + v.y);
	};

	o.subtract = function(v)
	{
		return vec2(o.x - v.x, o.y - v.y);
	};

	o.multiply = function(s)
	{
		return vec2(o.x * s, o.y * s);
	};

	o.divide = function(s)
	{
		return vec2(o.x / s, o.y / s);
	};

	o.dot = function(v)
	{
		return o.x * v.x + o.y * v.y;
	};

	o.cross = function(v)
	{
		return o.x * v.y - o.y * v.x;
	};

	o.sqrmag = function()
	{
		return o.x * o.x + o.y * o.y;
	}

	o.mag = function()
	{
		return Math.sqrt(o.sqrmag());
	};

	o.normalize = function()
	{
		var magnitude = o.mag();
		return vec2(magnitude ? o.x / magnitude : 0, 
					magnitude ? o.y / magnitude : 0);
	};

	o.perp = function(ccw)
	{
		return ccw ? vec2(-o.y, o.x) : vec2(o.y, -o.x);
	};

	o.rotate = function(angle)
	{
		var cos = Math.cos(angle);
		var sin = Math.sin(angle);

		var dx = o.x * cos - o.y * sin;
		var dy = o.x * sin + o.y * cos;

		return vec2(dx, dy);
	};

	o.scale = function(v)
	{
		return vec2(o.x * v.x, o.y * v.y);
	};

	o.projectOnto = function(v, origin)
	{
		var d = o.dot(v);
		return origin ? origin.add(v.multiply(d)) : v.multiply(d);
	};

	o.isZero = function()
	{
		return (x === 0) && (y === 0);
	};

	return o;
};



function edge(a, b)
{
	var e = b.subtract(a);
	var perp = e.perp(true);

	return {
		start: a, 
		end: b, 
		e: e,
		perp: perp
	};
};

function clamp(n, min, max)
{
	return Math.min(Math.max(n, min), max);
}

function crossSV(n, v)
{
	return vec2(-n * v.y, n * v.x);
}