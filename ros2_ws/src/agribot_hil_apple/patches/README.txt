IFRA_LinkAttacher, Apache-2.0, https://github.com/IFRA-Cranfield/IFRA_LinkAttacher
Pinned revision: b056289ba93ccb549db98926dcbb9679642d0c8d (humble).

ifra_multi_fixture.patch fixes per-pair attachment tracking, removes detached
entries, uses the actual joint name on detach, corrects a one-DOF damping index,
and locks the physics update mutex during joint edits. Optional SDF parameter
enable_gravity_on_attach defaults to false; the HIL scene opts in so dynamic
fruit stays parked until its tree fixture exists, then has normal gravity.
No world-pose teleport is added.

Detach pauses the world BEFORE acquiring the physics mutex and restores its
previous pause state after releasing the mutex. A real GDB trace showed the
inverse order deadlocks World::Update/ODE UpdateCollision against
Model::RemoveJoint/World::SetPaused when releasing a held dynamic fruit.
The physics mutex is also explicitly released before RemoveJoint acquires its
own world-update locks; service callbacks remain mutually exclusive.

Apply with git apply, then build linkattacher_msgs and ros2_linkattacher into
the private HIL overlay. The upstream copyright and Apache notice are retained.
