package nova.physics.wrapper.mc.forge.v18.wrapper.entity;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionConfiguration;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;
import net.minecraft.block.Block;
import net.minecraft.util.BlockPos;
import net.minecraft.world.chunk.Chunk;
import nova.core.component.Updater;
import nova.core.component.misc.Collider;
import nova.core.entity.Entity;
import nova.core.entity.component.RigidBody;
import nova.core.event.WorldEvent;
import nova.core.util.shape.Cuboid;
import nova.core.wrapper.mc.forge.v18.wrapper.block.world.BWWorld;
import nova.core.wrapper.mc.forge.v18.wrapper.entity.forward.MCEntityTransform;
import nova.internal.core.Game;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;

import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3f;
import java.util.HashMap;
import java.util.Map;

/**
 * JBullet physics handling
 * @author Calclavia
 */
public class PhysicsWorld implements Updater {

	private static final Map<net.minecraft.world.World, PhysicsWorld> WORLD_MAP = new HashMap<>();

	public static PhysicsWorld get(net.minecraft.world.World mcWorld) {
		if (!WORLD_MAP.containsKey(mcWorld)) {
			WORLD_MAP.put(mcWorld, new PhysicsWorld(mcWorld));
		}

		return WORLD_MAP.get(mcWorld);
	}

	public static void hookEvents() {
		Game.events().on(WorldEvent.Load.class).bind(evt -> get(((BWWorld) evt.world).world()));
		Game.events().on(WorldEvent.Unload.class).bind(evt -> WORLD_MAP.remove(((BWWorld) evt.world).world()));
	}

	private final net.minecraft.world.World MC_WORLD;
	private final DynamicsWorld PHYSICS_WORLD;

	private final Map<Vector2D, PhysicsChunk> chunks = new HashMap<>();

	public PhysicsWorld(net.minecraft.world.World mcWorld) {
		MC_WORLD = mcWorld;

		BroadphaseInterface broadphase = new DbvtBroadphase();
		CollisionConfiguration config = new DefaultCollisionConfiguration();
		CollisionDispatcher dispatcher = new CollisionDispatcher(config);
		ConstraintSolver solver = new SequentialImpulseConstraintSolver();

		PHYSICS_WORLD = new DiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
		PHYSICS_WORLD.setGravity(new Vector3f(0, -9.81f, 0));

		Game.syncTicker().add(this);
	}

	@Override
	public void update(double deltaTime) {
		PHYSICS_WORLD.stepSimulation((float) deltaTime);
	}

	public PhysicsChunk getChunk(int x, int z) {
		Vector2D key = new Vector2D(x, z);

		if (!chunks.containsKey(key)) {
			chunks.put(key, new PhysicsChunk((int) key.getX(), (int) key.getY()));
		}

		return chunks.get(key);
	}

	/**
	 * Simulates an entity and relevant chunks.
	 */
	public com.bulletphysics.dynamics.RigidBody simulateEntity(Entity entity) {
		com.bulletphysics.dynamics.RigidBody rigidBody = addEntity(entity);
		MCEntityTransform mcEntityTransform = entity.components.get(MCEntityTransform.class);
		getChunk(mcEntityTransform.wrapper.chunkCoordX, mcEntityTransform.wrapper.chunkCoordZ);
		return rigidBody;
	}

	private com.bulletphysics.dynamics.RigidBody addEntity(Entity entity) {
		Collider collider = entity.components.get(Collider.class);
		//TODO: Link the RBs
		RigidBody rigidBody = entity.components.get(RigidBody.class);
		Cuboid cuboid = collider.boundingBox.get();
		Vector3D position = entity.position();
		Rotation rotation = entity.rotation();

		BoxShape box = new BoxShape(new Vector3f((float) (cuboid.min.getX() + cuboid.max.getX()) / 2f, (float) (cuboid.min.getY() + cuboid.max.getY()) / 2f, (float) (cuboid.min.getZ() + cuboid.max.getZ()) / 2f));
		MotionState state = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f((float) rotation.getQ1(), (float) rotation.getQ2(), (float) rotation.getQ3(), (float) rotation.getQ0()), new Vector3f((float) position.getX(), (float) position.getY(), (float) position.getZ()), 0)));
		//TODO: Handle mass
		RigidBodyConstructionInfo info = new RigidBodyConstructionInfo(1, state, box);
		com.bulletphysics.dynamics.RigidBody entityRigidBody = new com.bulletphysics.dynamics.RigidBody(info);
		PHYSICS_WORLD.addRigidBody(entityRigidBody);
		return entityRigidBody;
	}

	public class PhysicsChunk {
		public final int x;
		public final int z;

		public PhysicsChunk(int x, int z) {
			this.x = x;
			this.z = z;

			initBlocks();
			initEntities();
		}

		private void initBlocks() {
			//TODO: Do not duplicate
			//TODO: Auto update block changes
			Chunk chunk = MC_WORLD.getChunkFromChunkCoords(x, z);
			int i = 0;
			for (int checkX = 0; checkX < 64; checkX++)
				for (int checkY = 0; checkY < 64; checkY++)
					for (int checkZ = 0; checkZ < 64; checkZ++) {
						Vector3D pos = new Vector3D(x + checkX, checkY, z + checkZ);
						BlockPos blockPos = new BlockPos(x + checkX, checkY, z + checkZ);
						Block block = chunk.getBlock(blockPos);

						if (!block.isAir(MC_WORLD, blockPos)) {
							//Generate colliders
							//TODO: The bounds are wrong
							BoxShape box = new BoxShape(new Vector3f((float) (block.getBlockBoundsMinX() + block.getBlockBoundsMaxX()) / 2f, (float) (block.getBlockBoundsMinY() + block.getBlockBoundsMaxY()) / 2f, (float) (block.getBlockBoundsMinZ() + block.getBlockBoundsMaxZ()) / 2f));
							MotionState state = new DefaultMotionState(new Transform(new Matrix4f(new Quat4f(0, 0, 0, 1), new Vector3f((float) pos.getX(), (float) pos.getY(), (float) pos.getZ()), 0)));
							RigidBodyConstructionInfo info = new RigidBodyConstructionInfo(0, state, box);
							com.bulletphysics.dynamics.RigidBody blockRigidBody = new com.bulletphysics.dynamics.RigidBody(info);
							blockRigidBody.setCollisionFlags(CollisionFlags.STATIC_OBJECT);
							PHYSICS_WORLD.addRigidBody(blockRigidBody);
							i++;
						}
					}

			System.out.println("Generated colliders: " + i);
		}

		private void initEntities() {

		}
	}
}
