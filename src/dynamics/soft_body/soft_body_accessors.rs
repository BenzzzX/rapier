//! Accessors and simple setters of a soft body: particles, elements, material, volume preservation, sleeping and enabling.
#[cfg(feature = "dim3")]
use super::SoftBodyDihedral;
#[cfg(feature = "fem")]
use super::SoftBodySolver;
use super::{
    SoftBody, SoftBodyCell, SoftBodyCellModel, SoftBodyEdge, SoftBodyMaterial, SoftBodyParticle,
    SoftVolumePiece,
};
use crate::dynamics::RigidBodyHandle;
use crate::math::{DIM, Real, Vector};
#[cfg(all(not(feature = "std"), feature = "dim2", feature = "experimental-soft-muscles"))]
#[allow(unused_imports)]
use simba::scalar::{ComplexField as _, RealField as _};

#[cfg(all(feature = "dim2", feature = "experimental-soft-muscles"))]
fn cell_trajectory_paths(
    cell: &SoftBodyCell,
    muscles: &[Option<super::SoftCellMuscle>; super::SOFT_CELL_MUSCLE_DIRECTIONS],
    start: &[Vector], end: &[Vector], duration: f64,
) -> Option<[Option<super::SoftCellMusclePath>; super::SOFT_CELL_MUSCLE_DIRECTIONS]> {
    let [a, b, c] = cell.vertices.map(|index| index as usize);
    let xy = |v: Vector| [v.x as f64, v.y as f64];
    let sub = |x: [f64; 2], y: [f64; 2]| [x[0] - y[0], x[1] - y[1]];
    let cross = |x: [f64; 2], y: [f64; 2]| x[0] * y[1] - x[1] * y[0];
    // Use the very same reference transform as the physical constraint;
    // initial particle coordinates can differ after a prior rest-shape change.
    let inverse_det = cell.inv_rest_matrix.determinant() as f64;
    if !inverse_det.is_finite() || inverse_det == 0.0 { return None; }
    let s1 = sub(xy(*start.get(b)?), xy(*start.get(a)?));
    let s2 = sub(xy(*start.get(c)?), xy(*start.get(a)?));
    let e1 = sub(xy(*end.get(b)?), xy(*end.get(a)?));
    let e2 = sub(xy(*end.get(c)?), xy(*end.get(a)?));
    let d1 = sub(e1, s1);
    let d2 = sub(e2, s2);
    // det(F(t)) is quadratic. Check its endpoints and interior minimum;
    // the dimensionless threshold is independent of world length units.
    let qa = cross(s1, s2) * inverse_det;
    let qb = (cross(d1, s2) + cross(s1, d2)) * inverse_det;
    let qc = cross(d1, d2) * inverse_det;
    if !qa.is_finite() || !qb.is_finite() || !qc.is_finite() ||
        qa <= 1.0e-8 || qa + qb + qc <= 1.0e-8 { return None; }
    if qc > 0.0 {
        let t = -qb / (2.0 * qc);
        if t > 0.0 && t < 1.0 && qa + t * (qb + t * qc) <= 1.0e-8 { return None; }
    }
    let mut paths = [None; super::SOFT_CELL_MUSCLE_DIRECTIONS];
    for (i, muscle) in muscles.iter().enumerate() {
        let Some(muscle) = muscle else { continue; };
        let coefficients = cell.inv_rest_matrix * muscle.direction;
        let g1 = coefficients.x as f64;
        let g2 = coefficients.y as f64;
        let q0 = [s1[0] * g1 + s2[0] * g2, s1[1] * g1 + s2[1] * g2];
        let q1 = [e1[0] * g1 + e2[0] * g2, e1[1] * g1 + e2[1] * g2];
        if !q0.iter().chain(q1.iter()).all(|x| x.is_finite()) { return None; }
        let delta = sub(q1, q0);
        let delta2 = delta[0] * delta[0] + delta[1] * delta[1];
        let q0_len2 = q0[0] * q0[0] + q0[1] * q0[1];
        let q1_len2 = q1[0] * q1[0] + q1[1] * q1[1];
        let max_real = Real::MAX as f64;
        if !q0_len2.is_finite() || !q1_len2.is_finite() || !delta2.is_finite() ||
            q0_len2.sqrt() > max_real || q1_len2.sqrt() > max_real || delta2.sqrt() / duration > max_real {
            return None;
        }
        let t = if delta2 > 0.0 { (-(q0[0] * delta[0] + q0[1] * delta[1]) / delta2).clamp(0.0, 1.0) } else { 0.0 };
        let q = [q0[0] + t * delta[0], q0[1] + t * delta[1]];
        if q[0] * q[0] + q[1] * q[1] <= 1.0e-12 { return None; }
        paths[i] = Some(super::SoftCellMusclePath {
            controls: [q0,
                [q0[0] + delta[0] / 3.0, q0[1] + delta[1] / 3.0],
                [q1[0] - delta[0] / 3.0, q1[1] - delta[1] / 3.0], q1],
            duration,
        });
    }
    Some(paths)
}

// A conservative cell test for a cubic Bezier target. Each control-point
// deformation F_i must be in the Frobenius ball |F_i - I| < 0.9. Bernstein
// weights are nonnegative and sum to one, so the entire curve stays in that
// convex ball and is orientation-preserving in exact arithmetic. This bound
// rejects many valid motions and is not a floating-point error certificate.
#[cfg(all(feature = "dim2", feature = "experimental-soft-muscles"))]
fn cell_cubic_trajectory_paths(
    cell: &SoftBodyCell,
    muscles: &[Option<super::SoftCellMuscle>; super::SOFT_CELL_MUSCLE_DIRECTIONS],
    vertices: [&[Vector]; 4], duration: f64,
) -> Option<[Option<super::SoftCellMusclePath>; super::SOFT_CELL_MUSCLE_DIRECTIONS]> {
    let [a, b, c] = cell.vertices.map(|index| index as usize);
    let inv = cell.inv_rest_matrix;
    let ix = inv.col(0);
    let iy = inv.col(1);
    let mut edges = [[[0.0_f64; 2]; 2]; 4];
    for (k, points) in vertices.iter().enumerate() {
        let (a, b, c) = (*points.get(a)?, *points.get(b)?, *points.get(c)?);
        let e1 = [b.x as f64 - a.x as f64, b.y as f64 - a.y as f64];
        let e2 = [c.x as f64 - a.x as f64, c.y as f64 - a.y as f64];
        edges[k] = [e1, e2];
        let f00 = e1[0] * ix.x as f64 + e2[0] * ix.y as f64;
        let f10 = e1[1] * ix.x as f64 + e2[1] * ix.y as f64;
        let f01 = e1[0] * iy.x as f64 + e2[0] * iy.y as f64;
        let f11 = e1[1] * iy.x as f64 + e2[1] * iy.y as f64;
        let error2 = (f00 - 1.0).powi(2) + f10 * f10 + f01 * f01 + (f11 - 1.0).powi(2);
        if !error2.is_finite() || error2 >= 0.81 { return None; }
    }
    let mut paths = [None; super::SOFT_CELL_MUSCLE_DIRECTIONS];
    for (i, muscle) in muscles.iter().enumerate() {
        let Some(muscle) = muscle else { continue; };
        let coeff = inv * muscle.direction;
        let mut controls = [[0.0; 2]; 4];
        for k in 0..4 {
            for axis in 0..2 {
                controls[k][axis] = edges[k][0][axis] * coeff.x as f64 + edges[k][1][axis] * coeff.y as f64;
            }
        }
        let max_real = Real::MAX as f64;
        if controls.iter().flatten().any(|v| !v.is_finite() || v.abs() > max_real / 4.0) { return None; }
        for k in 0..3 {
            for axis in 0..2 {
                if (controls[k + 1][axis] - controls[k][axis]).abs() * 3.0 / duration > max_real / 4.0 {
                    return None;
                }
            }
        }
        paths[i] = Some(super::SoftCellMusclePath { controls, duration });
    }
    Some(paths)
}

impl SoftBody {
    /// Whether inverted interior cells are ignored by self-contact recovery.
    pub fn ignores_inverted_cells_for_self_contacts(&self) -> bool {
        self.ignore_inverted_cells_for_self_contacts
    }

    /// Controls whether inverted internal cells stop self contacts. Surface
    /// crossings continue to participate in the normal tangle recovery.
    pub fn set_ignore_inverted_cells_for_self_contacts(&mut self, ignore: bool) {
        self.ignore_inverted_cells_for_self_contacts = ignore;
    }

    /// A counter bumped whenever the topology changes (a tear removed elements or duplicated
    /// particles): a renderer keying its mesh on the particles and elements rebuilds it when
    /// this value changes.
    pub fn topology_version(&self) -> u32 {
        self.topology_version
    }

    /// The particles of this soft body.
    pub fn particles(&self) -> &[SoftBodyParticle] {
        &self.particles
    }

    /// The number of particles of this soft body.
    pub fn num_particles(&self) -> usize {
        self.particles.len()
    }

    /// The world-space position of the `i`-th particle.
    pub fn particle_position(&self, i: usize) -> Vector {
        self.particles[i].position
    }

    /// The world-space positions of every particle.
    pub fn particle_positions(&self) -> impl ExactSizeIterator<Item = Vector> + '_ {
        self.particles.iter().map(|p| p.position)
    }

    /// The world-space velocity of the `i`-th particle.
    pub fn particle_velocity(&self, i: usize) -> Vector {
        self.particles[i].velocity
    }

    /// The world-space velocities of every particle.
    pub fn particle_velocities(&self) -> impl ExactSizeIterator<Item = Vector> + '_ {
        self.particles.iter().map(|p| p.velocity)
    }

    /// Teleports the `i`-th particle to `position` (no velocity change). For a pinned particle
    /// moving along a path, prefer [`Self::set_particle_kinematic_target`], which gives it the
    /// matching velocity so friction and contacts see the motion.
    pub fn set_particle_position(&mut self, i: usize, position: Vector) {
        self.particles[i].position = position;
        self.particles[i].next_position = None;
        self.positions_modified = true;
        self.modified = true;
    }

    /// Sets the velocity of the `i`-th particle. A pinned particle keeps moving at that velocity
    /// (a velocity-driven kinematic particle) until it is set again or given a kinematic target.
    pub fn set_particle_velocity(&mut self, i: usize, velocity: Vector) {
        self.particles[i].velocity = velocity;
        self.particles[i].next_position = None;
        self.modified = true;
    }

    /// Moves the pinned `i`-th particle to `position` over the next step (with the matching
    /// velocity, so dragged cloth and friction see the motion), then holds it there until the next
    /// target or velocity is set. Ignored for a free particle.
    pub fn set_particle_kinematic_target(&mut self, i: usize, position: Vector) {
        let particle = &mut self.particles[i];
        if particle.inv_mass == 0.0 {
            particle.next_position = Some(position);
            // A target away from the particle is a motion: the body must be awake to follow it.
            if position != particle.position {
                self.modified = true;
            }
        }
    }

    /// The distance constraints of this soft body.
    pub fn edges(&self) -> &[SoftBodyEdge] {
        &self.edges
    }

    /// Atomically replaces the fixed-target edge actuators. The sum of their
    /// weighted force caps may not exceed `total_effort`; a missing entry
    /// leaves that edge passive. Commands persist until replaced or topology
    /// changes. FEM and tearing are unsupported by this experimental path.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-muscles"))]
    pub fn set_edge_muscles(
        &mut self,
        muscles: &[Option<super::SoftEdgeMuscle>],
        total_effort: Real,
    ) -> bool {
        if muscles.len() != self.edges.len() || !total_effort.is_finite() || total_effort < 0.0
            || self.uses_fem() || self.material.tears()
            || (muscles.iter().any(Option::is_some) && self.cells.iter().any(|cell| cell.muscles.iter().any(Option::is_some)))
        {
            return false;
        }
        let mut remaining = total_effort as f64;
        for muscle in muscles.iter().flatten() {
            if !muscle.target_length.is_finite() || muscle.target_length <= 0.0
                || !muscle.stiffness.is_finite() || muscle.stiffness < 0.0
                || !muscle.damping.is_finite() || muscle.damping < 0.0
                || !muscle.maximum_force.is_finite() || muscle.maximum_force < 0.0
                || !muscle.effort_weight.is_finite() || muscle.effort_weight <= 0.0
            {
                return false;
            }
            remaining -= (muscle.maximum_force as f64) * (muscle.effort_weight as f64);
            if !remaining.is_finite() || remaining < 0.0 {
                return false;
            }
        }
        for (edge, muscle) in self.edges.iter_mut().zip(muscles) {
            edge.muscle = muscle.map(|value| (value, self.topology_version));
        }
        self.modified = true;
        true
    }

    /// Atomically replaces up to three experimental actuators per cell. Their
    /// force densities are integrated over reference areas and must fit `total_effort`. This
    /// path requires nonplastic, nontearing 2D elastic constraint cells, and
    /// does not support FEM or simultaneous edge actuators. A target moves at
    /// its signed rate toward its positive limit, then holds. Disabled or
    /// fully frozen bodies do not advance the target. All cells use one batch
    /// clock, including cells whose particles are temporarily all pinned.
    /// Topology changes invalidate the stored commands.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-muscles"))]
    pub fn set_cell_muscles(
        &mut self,
        muscles: &[[Option<super::SoftCellMuscle>; super::SOFT_CELL_MUSCLE_DIRECTIONS]],
        total_effort: Real,
    ) -> bool {
        if muscles.len() != self.cells.len() || !total_effort.is_finite() || total_effort < 0.0
            || self.uses_fem() || self.material.tears() || self.material.plastic_yield > 0.0
            || self.cell_model == SoftBodyCellModel::Volume
            || !self.material.young_modulus.is_finite() || self.material.young_modulus <= 0.0
            || (muscles.iter().flatten().any(Option::is_some) && self.edges.iter().any(|edge| edge.muscle.is_some()))
        {
            return false;
        }
        let mut remaining = total_effort as f64;
        let Some(next_revision) = self.cell_muscle_revision.checked_add(1) else { return false; };
        for (cell, cell_muscles) in self.cells.iter().zip(muscles) {
            for muscle in cell_muscles.iter().flatten() {
                let length_squared = muscle.direction.length_squared();
                if !length_squared.is_finite() || (length_squared - 1.0).abs() > 1.0e-4
                    || !muscle.target_stretch.is_finite() || muscle.target_stretch <= 0.0
                    || !muscle.target_rate.is_finite() || !muscle.target_limit.is_finite()
                    || muscle.target_limit <= 0.0
                    || (muscle.target_rate > 0.0 && muscle.target_limit < muscle.target_stretch)
                    || (muscle.target_rate < 0.0 && muscle.target_limit > muscle.target_stretch)
                    || !muscle.stiffness_density.is_finite() || muscle.stiffness_density < 0.0
                    || !muscle.damping_density.is_finite() || muscle.damping_density < 0.0
                    || !muscle.force_density.is_finite() || muscle.force_density < 0.0
                    || !cell.rest_volume.is_finite() || cell.rest_volume <= 0.0
                    || !cell.stiffness_scale.is_finite() || cell.stiffness_scale <= 0.0
                {
                    return false;
                }
                remaining -= (muscle.force_density as f64) * (cell.rest_volume as f64);
                if !remaining.is_finite() || remaining < 0.0 {
                    return false;
                }
            }
        }
        for (cell, cell_muscles) in self.cells.iter_mut().zip(muscles) {
            cell.muscles = cell_muscles.map(|muscle| muscle.map(|value|
                (value, self.topology_version, None)));
        }
        self.cell_muscle_clock = muscles.iter().flatten().any(Option::is_some)
            .then_some((self.topology_version, 0.0));
        self.cell_muscle_revision = next_revision;
        self.modified = true;
        true
    }

    /// The batch's current commanded stretch, not a measurement of strain or
    /// evidence that this cell has participated in the solve.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-muscles"))]
    pub fn cell_muscle_target(&self, cell: usize, direction: usize) -> Option<f64> {
        let (clock_version, elapsed) = self.cell_muscle_clock?;
        let (command, version, path) = self.cells.get(cell)?.muscles.get(direction)?.as_ref()?;
        (clock_version == self.topology_version && *version == clock_version)
            .then(|| command.target_at_elapsed(*path, elapsed))
    }

    /// Revision and elapsed solver time of the current experimental batch.
    /// A frozen body does not advance this clock. Identity additionally needs
    /// the owning soft-body handle; a different body may reuse a revision.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-muscles"))]
    pub fn cell_muscle_clock_state(&self) -> Option<(u64, f64)> {
        let (version, elapsed) = self.cell_muscle_clock?;
        (version == self.topology_version && elapsed.is_finite()).then_some((self.cell_muscle_revision, elapsed))
    }

    /// Atomically installs muscles driven by one linear vertex trajectory.
    /// All cells use the same endpoints and duration; each cell's interpolated
    /// deformation must remain positively oriented throughout the interval.
    /// This validates target geometry, not the simulated body's collisions,
    /// self-intersections, inversion, or ability to attain those targets.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-muscles"))]
    pub fn set_cell_muscle_trajectory(
        &mut self,
        muscles: &[[Option<super::SoftCellMuscle>; super::SOFT_CELL_MUSCLE_DIRECTIONS]],
        start: &[Vector], end: &[Vector], duration: Real, total_effort: Real,
    ) -> bool {
        if start.len() != self.particles.len() || end.len() != start.len() ||
            muscles.len() != self.cells.len() || !duration.is_finite() || duration <= 0.0 ||
            start.iter().chain(end.iter()).any(|p| !p.is_finite()) { return false; }
        for (cell, commands) in self.cells.iter().zip(muscles) {
            if cell_trajectory_paths(cell, commands, start, end, duration as f64).is_none() { return false; }
        }
        if !self.set_cell_muscles(muscles, total_effort) { return false; }
        for (cell, commands) in self.cells.iter_mut().zip(muscles) {
            // Identical immutable inputs were validated before any mutation.
            let paths = cell_trajectory_paths(cell, commands, start, end, duration as f64).unwrap();
            for (muscle, path) in cell.muscles.iter_mut().zip(paths) {
                if let Some((_, _, stored_path)) = muscle { *stored_path = path; }
            }
        }
        true
    }

    /// Install one cubic Bezier target shared by all vertices and cells.
    /// The conservative per-control deformation bound excludes inversion in
    /// exact arithmetic; it is not a numerical or physical safety guarantee.
    /// Boundary intersection, topology and transition continuity are the
    /// caller's responsibility. All controls use the material reference frame.
    #[cfg(all(feature = "dim2", feature = "experimental-soft-muscles"))]
    pub fn set_cell_muscle_cubic_trajectory(
        &mut self,
        muscles: &[[Option<super::SoftCellMuscle>; super::SOFT_CELL_MUSCLE_DIRECTIONS]],
        vertices: [&[Vector]; 4], duration: Real, total_effort: Real,
    ) -> bool {
        if vertices.iter().any(|p| p.len() != self.particles.len() || p.iter().any(|v| !v.is_finite())) ||
            muscles.len() != self.cells.len() || !duration.is_finite() || duration <= 0.0 { return false; }
        for (cell, commands) in self.cells.iter().zip(muscles) {
            if cell_cubic_trajectory_paths(cell, commands, vertices, duration as f64).is_none() { return false; }
        }
        if !self.set_cell_muscles(muscles, total_effort) { return false; }
        for (cell, commands) in self.cells.iter_mut().zip(muscles) {
            let paths = cell_cubic_trajectory_paths(cell, commands, vertices, duration as f64).unwrap();
            for (muscle, path) in cell.muscles.iter_mut().zip(paths) {
                if let Some((_, _, stored_path)) = muscle { *stored_path = path; }
            }
        }
        true
    }

    /// The dihedral bending constraints of this soft body.
    #[cfg(feature = "dim3")]
    pub fn dihedrals(&self) -> &[SoftBodyDihedral] {
        &self.dihedrals
    }

    /// The simplex cells (triangles in 2D, tetrahedra in 3D) of this soft body.
    pub fn cells(&self) -> &[SoftBodyCell] {
        &self.cells
    }

    /// The boundary of this soft body's computational mesh: segments in 2D, triangles in 3D
    /// (particle indices), oriented outward. This is what the volume constraint integrates over; the
    /// geometry the body collides through is its [`crate::dynamics::SoftCollisionMesh`].
    pub fn boundary(&self) -> &[[u32; DIM]] {
        &self.boundary
    }

    /// The material (per-family stiffness and damping) of this soft body.
    pub fn material(&self) -> &SoftBodyMaterial {
        &self.material
    }

    /// Sets the material of this soft body.
    pub fn set_material(&mut self, material: SoftBodyMaterial) {
        self.material = material;
        self.modified = true;
    }

    /// Mutable access to the material of this soft body.
    pub fn material_mut(&mut self) -> &mut SoftBodyMaterial {
        self.modified = true;
        &mut self.material
    }

    /// The constitutive model of this soft body's cells.
    pub fn cell_model(&self) -> SoftBodyCellModel {
        self.cell_model
    }

    /// Which solver simulates this soft body's elasticity.
    #[cfg(feature = "fem")]
    pub fn solver(&self) -> SoftBodySolver {
        self.solver
    }

    /// Selects the solver simulating this soft body's elasticity (see [`SoftBodySolver`]).
    #[cfg(feature = "fem")]
    pub fn set_solver(&mut self, solver: SoftBodySolver) {
        self.solver = solver;
        self.modified = true;
    }

    /// Whether this soft body's elasticity is solved by the FEM path.
    #[inline]
    pub(crate) fn uses_fem(&self) -> bool {
        #[cfg(feature = "fem")]
        {
            self.solver == SoftBodySolver::Fem
        }
        #[cfg(not(feature = "fem"))]
        {
            false
        }
    }

    /// Whether the area/volume preservation constraints are enabled (see [`Self::volume_pieces`]).
    pub fn volume_preservation_enabled(&self) -> bool {
        self.volume_preservation
    }

    /// Enables or disables area/volume preservation: one constraint per volume piece (see
    /// [`Self::volume_pieces`]); it stays disabled for a body without any.
    pub fn enable_volume_preservation(&mut self, enabled: bool) {
        self.volume_preservation = enabled && !self.volume_pieces.is_empty();
        self.modified = true;
    }

    /// The pieces of material enclosed by a closed boundary, each with its own volume constraint
    /// so squeezing one does not inflate another. Pieces joined by no element (appended bodies,
    /// torn-apart halves) are separate; the holes or cavities of a piece belong to it.
    pub fn volume_pieces(&self) -> &[SoftVolumePiece] {
        &self.volume_pieces
    }

    /// The signed area (2D) or volume (3D) enclosed by the volume pieces at rest (`0.0` for a
    /// body without a closed boundary).
    pub fn rest_volume(&self) -> Real {
        self.volume_pieces
            .iter()
            .map(|piece| piece.rest_volume)
            .sum()
    }

    /// The signed area (2D) or volume (3D) currently enclosed by the volume pieces.
    pub fn volume(&self) -> Real {
        self.volume_pieces
            .iter()
            .map(|piece| piece.volume(self))
            .sum()
    }

    /// The multiplier applied to each volume piece's rest volume to obtain its
    /// volume-preservation target (`> 1` inflates the body).
    pub fn volume_factor(&self) -> Real {
        self.volume_factor
    }

    /// Sets the multiplier applied to each volume piece's rest volume to obtain its
    /// volume-preservation target (`> 1` inflates the body).
    pub fn set_volume_factor(&mut self, factor: Real) {
        self.volume_factor = factor;
        self.modified = true;
    }

    /// The particle spacing of this soft body: what the impact-adaptive substeps bound the
    /// travel per substep by, and the default contact skin of its collision meshes' colliders.
    pub fn particle_radius(&self) -> Real {
        self.particle_radius
    }

    /// The rigid body standing for this soft body in the islands and holding the colliders of its
    /// surface (invalid until inserted in a set): joints, forces and impulses applied to it act on
    /// the body as a whole. A tear splitting the body moves each of its joints to the piece closest
    /// to its anchor in the rest shape (see [`crate::dynamics::SoftBodyTearEvent::moved_joints`]).
    /// Its pose is recomputed from the particles at every step, so moving it has no effect, and
    /// removing it dissolves its whole-body cluster like any cluster proxy.
    pub fn root_body(&self) -> RigidBodyHandle {
        self.root_body
    }

    /// The soft body this one was split off from by a tear or a cut (`None` for a user-inserted
    /// body): the piece that kept the handle (see [`crate::dynamics::SoftBodyTearEvent::pieces`]).
    /// Informational only; the handle dangles once that body is removed.
    pub fn origin(&self) -> Option<crate::dynamics::SoftBodyHandle> {
        self.origin
    }

    /// The soft bodies split off from this one by tears and cuts, in creation order (a piece
    /// torn off a piece is listed by that piece, not here). Informational: the engine never
    /// reads it, and a handle dangles once that body is removed.
    pub fn pieces(&self) -> &[crate::dynamics::SoftBodyHandle] {
        &self.pieces
    }

    /// The dynamics settings of the particles (damping, gravity scale, sleeping...).
    pub fn particle_settings(&self) -> &super::SoftBodyParticleSettings {
        &self.particle_settings
    }

    /// Sets the extra internal PGS iterations per substep requested for this body's island
    /// component (see [`super::SoftBodyParticleSettings::additional_pgs_iterations`]); takes
    /// effect at the next step.
    pub fn set_additional_pgs_iterations(&mut self, iterations: usize) {
        self.particle_settings.additional_pgs_iterations = iterations;
    }

    /// The number of parallel solver colors used by this soft body's elements (informational:
    /// elements of a same color share no particle and are solved concurrently).
    pub fn num_solver_colors(&self) -> usize {
        self.num_colors as usize + self.has_overflow_color as usize
    }

    /// The mass-weighted center of the particles (from the cached positions).
    pub fn center_of_mass(&self) -> Vector {
        let mut com = Vector::ZERO;
        let mut mass = 0.0;
        for p in &self.particles {
            com += p.position * p.mass;
            mass += p.mass;
        }
        if mass > 0.0 { com / mass } else { com }
    }

    /// The total nominal mass of the particles.
    pub fn mass(&self) -> Real {
        self.particles.iter().map(|p| p.mass).sum()
    }

    /// Whether this soft body is currently sleeping (a soft body sleeps and wakes as a unit,
    /// with the island of the rigid bodies it touches). Updated at the end of every step, and
    /// by [`Self::wake_up`].
    pub fn is_sleeping(&self) -> bool {
        self.sleeping
    }

    /// Wakes this soft body up (effective at the start of the next step, together with the
    /// island of the bodies it touches).
    pub fn wake_up(&mut self) {
        self.sleeping = false;
        self.modified = true;
    }

    /// Whether this soft body takes part in the simulation. A disabled soft body (by
    /// [`Self::set_enabled`], or by the NaN quarantine: `PhysicsPipeline::quarantine`) is
    /// left where it is, without colliders or constraints, until it is enabled again.
    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    /// Enables or disables this soft body (effective at the start of the next step). Re-enabling
    /// a quarantined body: set its non-finite particles right first
    /// ([`Self::set_particle_position`]), they are not repaired.
    pub fn set_enabled(&mut self, enabled: bool) {
        if self.enabled != enabled {
            self.enabled = enabled;
            self.modified = true;
        }
    }

    /// Whether every particle's state is finite.
    pub(crate) fn is_finite(&self) -> bool {
        self.particles.iter().all(|p| {
            p.position.is_finite()
                && p.velocity.is_finite()
                && p.force.is_finite()
                && p.next_position.is_none_or(|t| t.is_finite())
        })
    }

    /// Neutralizes the dynamics of a non-finite body (velocities, forces, targets); the
    /// positions are left as they are.
    pub(crate) fn sanitize_dynamics(&mut self) {
        for p in &mut self.particles {
            p.velocity = Vector::ZERO;
            p.force = Vector::ZERO;
            p.next_position = None;
        }
    }
}
