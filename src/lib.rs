#![allow(improper_ctypes)] // FIXME?

#[macro_use]
extern crate cpp;
extern crate libc;

use libc::{c_int, c_short};
use std::ops::{Deref, DerefMut};

cpp!{{
    #include "btBulletDynamicsCommon.h"
}}

pub type Scalar = f32;
pub type Vector = [Scalar; 4];
pub type Quaternion = [Scalar; 4];
pub type Matrix = [Scalar; 12];
pub type Transform = [Scalar; 16];

// ++++++++++++++++++++ Ptr ++++++++++++++++++++

pub struct Ptr<T> {
    ptr: *mut T,
}

impl<T> Deref for Ptr<T> {
    type Target = T;
    fn deref(&self) -> &Self::Target { unsafe { &*self.ptr } }
}

impl<T> DerefMut for Ptr<T> {
    fn deref_mut(&mut self) -> &mut Self::Target { unsafe { &mut *self.ptr } }
}

// TODO impl<T> Drop for Ptr<T> {

// ++++++++++++++++++++ DynamicsWorld ++++++++++++++++++++

pub enum DynamicsWorld {}

impl DynamicsWorld {
    pub unsafe fn new() -> Ptr<Self> {
        cpp!{[] -> Ptr<DynamicsWorld> as "btDiscreteDynamicsWorld *" {
            auto broadphase = new btDbvtBroadphase();
            auto collision_cfg = new btDefaultCollisionConfiguration();
            auto dispatcher = new btCollisionDispatcher(collision_cfg);
            auto solver = new btSequentialImpulseConstraintSolver();

            auto ptr = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collision_cfg);

            return ptr;
        }}
    }
    pub unsafe fn delete(&self) {
        let mut ptr = self;
        cpp!{[mut ptr as "btDiscreteDynamicsWorld *"]{
            auto solver = ptr->getConstraintSolver();
            auto dispatcher = dynamic_cast<btCollisionDispatcher *>(ptr->getCollisionWorld()->getDispatcher());
            auto collision_cfg = dispatcher->getCollisionConfiguration();
            auto broadphase = ptr->getCollisionWorld()->getBroadphase();

            delete ptr;
            delete solver;
            delete dispatcher;
            delete collision_cfg;
            delete broadphase;
        }}
    }

    pub unsafe fn step_simulation(
        &self,
        dt: Scalar,
        max_sub_steps: c_int,
        fixed_step: Scalar,
    ) {
        let mut ptr = self;
        cpp!{[mut ptr as "btDiscreteDynamicsWorld *", dt as "btScalar", max_sub_steps as "int", fixed_step as "btScalar"]{
            ptr->stepSimulation(dt, max_sub_steps, fixed_step);
        }}
    }

    pub unsafe fn set_gravity(&self, v: Vector) {
        let mut ptr = self;
        cpp!{[mut ptr as "btDiscreteDynamicsWorld *", v as "btVector3"]{
            ptr->setGravity(v);
        }}
    }

    pub unsafe fn add_rigid_body(
        &self,
        mut body: &RigidBody,
        group: c_short,
        mask: c_short,
    ) {
        let mut ptr = self;
        cpp!{[
            mut ptr as "btDiscreteDynamicsWorld *", 
            mut body as "btRigidBody *",
            group as "short", 
            mask as "short"
        ]{
            // TODO make use of group and mask
            ptr->addRigidBody(body);
        }}
    }
    pub unsafe fn remove_rigid_body(&self, mut body: &RigidBody) {
        let mut ptr = self;
        cpp!{[mut ptr as "btDiscreteDynamicsWorld *", mut body as "btRigidBody *"]{
            ptr->removeRigidBody(body);
        }}
    }
}

// ++++++++++++++++++++ TriangleMesh ++++++++++++++++++++

/// TODO
///
/// * support 16 bit indices?
pub enum TriangleMesh {}

impl TriangleMesh {
    pub unsafe fn new(prealloc_verts: c_int, prealloc_indices: c_int) -> Ptr<TriangleMesh> {
        cpp!{[prealloc_verts as "int", prealloc_indices as "int"] -> Ptr<TriangleMesh> as "btTriangleMesh *" {
            auto ptr = new btTriangleMesh();
            ptr->preallocateVertices(prealloc_verts);
            ptr->preallocateIndices(prealloc_indices);
        }}
    }
    pub unsafe fn delete(&self) {
        let ptr = self;
        cpp!{[ptr as "btTriangleMesh *"]{
            delete ptr;
        }}
    }

    pub unsafe fn add_triangle(&self, v0: Vector, v1: Vector, v2: Vector) {
        let mut ptr = self;
        cpp!{[mut ptr as "btTriangleMesh *", v0 as "btVector3", v1 as "btVector3", v2 as "btVector3"]{
            ptr->addTriangle(v0, v1, v2);
        }}
    }
    pub unsafe fn add_triangle_indices(&self, i0: c_int, i1: c_int, i2: c_int) {
        let mut ptr = self;
        cpp!{[mut ptr as "btTriangleMesh *", i0 as "int", i1 as "int", i2 as "int"]{
            ptr->addTriangleIndices(i0, i1, i2);
        }}
    }
}

// ++++++++++++++++++++ CollisionShape ++++++++++++++++++++

pub enum CollisionShape {}

impl CollisionShape {
    pub unsafe fn new_static_plane(normal: Vector, constant: Scalar) -> Ptr<Self> {
        cpp!{[normal as "btVector3", constant as "float"] -> Ptr<CollisionShape> as "btCollisionShape *" {
            return new btStaticPlaneShape(normal, constant);
        }}
    }
    pub unsafe fn new_capsule(radius: Scalar, height: Scalar) -> Ptr<Self> {
        cpp!{[radius as "float", height as "float"] -> Ptr<CollisionShape> as "btCollisionShape *" {
            return new btCapsuleShape(radius, height); 
        }}
    }
    pub unsafe fn new_sphere(radius: Scalar) -> Ptr<Self> {
        cpp!{[radius as "float"] -> Ptr<CollisionShape> as "btCollisionShape *" {
            return new btSphereShape(radius); 
        }}
    }
    pub unsafe fn new_triangle_mesh(mut mesh: Ptr<TriangleMesh>) -> Ptr<Self> {
        cpp!{[mut mesh as "btTriangleMesh *"] -> Ptr<CollisionShape> as "btCollisionShape *" {
            return new btBvhTriangleMeshShape(mesh, true);
        }}
    }
    pub unsafe fn delete(&self) {
        let ptr = self;
        cpp!{[ptr as "btCollisionShape *"]{
            btStridingMeshInterface const* tri_mesh_ptr = nullptr;
            auto tri_mesh_shape_ptr = dynamic_cast<btTriangleMeshShape const*>(ptr);
            if (tri_mesh_shape_ptr) {
                tri_mesh_ptr = tri_mesh_shape_ptr->getMeshInterface();
            }

            delete ptr;
            if (tri_mesh_ptr) {
                delete tri_mesh_ptr;
            }
        }}
    }

    pub unsafe fn calculate_local_inertia(&self, mass: Scalar) -> Vector {
        let ptr = self;
        cpp!{[ptr as "btCollisionShape *", mass as "btScalar"] -> Vector as "btVector3" {
            btVector3 inertia;
            ptr->calculateLocalInertia(mass, inertia);
            return inertia;
        }}
    }
    pub unsafe fn set_margin(&self, v: Scalar) {
        let mut ptr = self;
        cpp!{[mut ptr as "btCollisionShape *", v as "btScalar"]{
            ptr->setMargin(v);
        }}
    }

    pub unsafe fn set_user_index(&self, i: c_int) {
        let mut ptr = self;
        cpp!{[mut ptr as "btCollisionShape *", i as "int"]{
            ptr->setUserIndex(i);
        }}
    }
    pub unsafe fn user_index(&self) -> c_int {
        let ptr = self;
        cpp!{[ptr as "btCollisionShape *"] -> c_int as "int"{
            return ptr->getUserIndex();
        }}
    }
}

// ++++++++++++++++++++ MotionState ++++++++++++++++++++

pub enum MotionState {}

impl MotionState {
    pub unsafe fn identity() -> Ptr<Self> {
        cpp!{[] -> Ptr<MotionState> as "btMotionState *" {
            return new btDefaultMotionState();
        }}
    }
    pub unsafe fn new(t: Transform) -> Ptr<Self> {
        cpp!{[t as "btTransform"] -> Ptr<MotionState>  as "btMotionState *" {
            return new btDefaultMotionState(t);
        }}
    }
    pub unsafe fn delete(&self) {
        let ptr = self;
        cpp!{[ptr as "btMotionState *"]{
            delete ptr;
        }}
    }
}

// ++++++++++++++++++++ RigidBodyConstructionInfo ++++++++++++++++++++

#[repr(C)]
pub struct RigidBodyConstructionInfo<'a> {
    pub mass: Scalar,
    pub motion_state: Ptr<MotionState>,
    pub start_world_transform: Transform,
    pub collision_shape: &'a CollisionShape,
    pub local_inertia: Vector,
    pub linear_damping: Scalar,
    pub angular_damping: Scalar,
    pub friction: Scalar,
    pub rolling_friction: Scalar,
    pub spinning_friction: Scalar,
    pub restitution: Scalar,
    pub linear_sleeping_threshold: Scalar,
    pub angular_sleeping_threshold: Scalar,
    /// TODO what type?
    pub additional_damping: bool,
    pub additional_damping_factor: Scalar,
    pub additional_linear_damping_threshold_sqr: Scalar,
    pub additional_angular_damping_threshold_sqr: Scalar,
    pub additional_angular_damping_factor: Scalar,
}

impl<'a> RigidBodyConstructionInfo<'a> {
    pub unsafe fn new(
        mass: Scalar,
        mut motion_state: Ptr<MotionState>,
        mut shape: &'a CollisionShape,
        local_inertia: Vector,
    ) -> Self {
        cpp!{[mass as "btScalar", mut motion_state as "btMotionState *", mut shape as "btCollisionShape *", local_inertia as "btVector3"] -> RigidBodyConstructionInfo as "btRigidBody::btRigidBodyConstructionInfo" {
            btRigidBody::btRigidBodyConstructionInfo info(mass, motion_state, shape, local_inertia);
            return info;
        }}
    }
    pub unsafe fn delete(&self) { self.motion_state.delete(); }
}

// ++++++++++++++++++++ RigidBody ++++++++++++++++++++

pub enum RigidBody {}

impl RigidBody {
    pub unsafe fn new(info: RigidBodyConstructionInfo) -> Ptr<Self> {
        // TODO pass info by reference?
        cpp!{[info as "btRigidBody::btRigidBodyConstructionInfo"] -> Ptr<RigidBody> as "btRigidBody *" {
            return new btRigidBody(info);
        }}
    }
    pub unsafe fn delete(&self) {
        let ptr = self;
        cpp!{[ptr as "btRigidBody *"]{
            auto motion_state = ptr->getMotionState();

            delete ptr;
            delete motion_state;
        }}
    }

    // ++++++++++++++++++++ CollisionObject ++++++++++++++++++++

    pub unsafe fn set_collision_shape(&self, mut s: &CollisionShape) {
        let mut ptr = self;
        cpp!{[mut ptr as "btRigidBody *", mut s as "btCollisionShape *"]{
            ptr->setCollisionShape(s);
        }}
    }

    pub unsafe fn set_world_transform(&self, t: Transform) {
        let mut ptr = self;
        cpp!{[mut ptr as "btRigidBody *", t as "btTransform"]{
            ptr->setWorldTransform(t);
        }}
    }
    pub unsafe fn world_transform(&self) -> Transform {
        let mut ptr = self;
        cpp!{[mut ptr as "btRigidBody *"] -> Transform as "btTransform"{
            return ptr->getWorldTransform();
        }}
    }

    pub unsafe fn set_user_index(&self, i: c_int) {
        let mut ptr = self;
        cpp!{[mut ptr as "btRigidBody *", i as "int"]{
            ptr->setUserIndex(i);
        }}
    }
    pub unsafe fn user_index(&self) -> c_int {
        let ptr = self;
        cpp!{[ptr as "btRigidBody *"] -> c_int as "int"{
            return ptr->getUserIndex();
        }}
    }

    // ++++++++++++++++++++ RigidBody ++++++++++++++++++++

    pub unsafe fn set_linear_velocity(&self, v: Vector) {
        let mut ptr = self;
        cpp!{[mut ptr as "btRigidBody *", v as "btVector3"]{
            ptr->setLinearVelocity(v);
        }}
    }
    pub unsafe fn linear_velocity(&self) -> Vector {
        let ptr = self;
        cpp!{[ptr as "btRigidBody *"] -> Vector as "btVector3"{
            return ptr->getLinearVelocity();
        }}
    }

    pub unsafe fn set_angular_velocity(&self, v: Vector) {
        let mut ptr = self;
        cpp!{[mut ptr as "btRigidBody *", v as "btVector3"]{
            ptr->setAngularVelocity(v);
        }}
    }
    pub unsafe fn angular_velocity(&self) -> Vector {
        let ptr = self;
        cpp!{[ptr as "btRigidBody *"] -> Vector as "btVector3"{
            return ptr->getAngularVelocity();
        }}
    }

    //     pub unsafe fn translate(&self, v: Vector){
    //         let body = self.clone();
    //         cpp!{[body as "RigidBody", v as "btVector3"]{
    //             body.ptr->translate(v);
    //         }}
    //     }
}
