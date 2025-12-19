#include <iostream>

#include <stark>
#include <filesystem>
#include "paths.h"
#include "rb_constraint_test_scenes.h"

#include <VolumeSampling.h>

using MagneticMaterial = stark::MagneticMaterial;

float mu_neodymium = 1.32e-6;
float mu_nickel = 1.26e-4;
float mu_iron = 6.3e-3;


void sample_magnetic_object(const std::vector<Eigen::Vector3d>& vertices, const std::vector<unsigned int>& faces, const double radius, std::vector<Eigen::Vector3d>& magnetic_pos_samples)
{
	Utilities::VolumeSampling::sampleMesh(vertices.size(), vertices.data(), faces.size() / 3, faces.data(), radius,
										  nullptr, {50, 50, 50}, false, 0, magnetic_pos_samples);

	std::cout << "Created N=" << magnetic_pos_samples.size() << " samples for the magnetic needles!" << std::endl;
}

void sample_magnetic_object(const stark::Mesh<3> mesh, const double radius, std::vector<Eigen::Vector3d>& magnetic_pos_samples)
{
	std::vector<unsigned int> casted_faces;
	for (const auto& face : mesh.conn)
	{
		casted_faces.push_back(face[0]);
		casted_faces.push_back(face[1]);
		casted_faces.push_back(face[2]);
	}

	sample_magnetic_object(mesh.vertices, casted_faces, radius, magnetic_pos_samples);
}

void sample_magnetic_object(const std::vector<Eigen::Vector3d>& vertices, const std::vector<std::array<int,3>>& faces, const double radius, std::vector<Eigen::Vector3d>& magnetic_pos_samples)
{
	std::vector<unsigned int> casted_faces;
	for (const auto& face : faces)
	{
		casted_faces.push_back(face[0]);
		casted_faces.push_back(face[1]);
		casted_faces.push_back(face[2]);
	}

	sample_magnetic_object(vertices, casted_faces, radius, magnetic_pos_samples);
}

void magnetic_attraction_ekw()
{
	// Parameters
	double mass = 0.1; // kg
	double size = 0.1; // m
	double contact_thickness = 0.00005; // m

	// Default example settings
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "magnetic_attraction_ekw";
	settings.output.output_directory = OUTPUT_PATH + "/magnetic_attraction_ekw";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 1.0;
	settings.debug.symx_check_for_NaNs = true;
	settings.simulation.gravity = { 0.0, 0.0, -9.81 };
	settings.simulation.magnetic_method = stark::MagneticMethod::DipoleMoment;
	settings.simulation.max_time_step_size = 0.001;
	settings.simulation.use_adaptive_time_step = false;
	settings.newton.max_line_search_iterations = 100;
	settings.newton.max_newton_iterations = 1000;
	settings.output.fps = 240;

	stark::Simulation simulation(settings);

	simulation.interactions->contact->set_global_params(
		stark::EnergyFrictionalContact::GlobalParams()
			.set_default_contact_thickness(contact_thickness)
			.set_friction_enabled(true)
			.set_friction_stick_slide_threshold(0.001)
			.set_min_contact_stiffness(1e14)
	);

	// Adding more realistic magnetic parameters
	const double B_r = 1.48; // Tesla - residual flux density of N52 grade magnet
	const double mu_0 = 4*3.14*10e-7; // H/m - permeability of free-space
	const double magnetization = B_r / mu_0;
    const double magnetic_strength = magnetization*std::pow(size,3); // A-m^2 - dipole moment

	// Add cube magnets
	int n = 2;
	for (int i = 0; i < n; i++) {
		auto [vertices, triangles, handler] = simulation.presets->rigidbodies->add_box("box", mass, size);
		handler.rigidbody.add_translation({ 2.5*i*size, 0.0, 0.0 });
		auto magnetic_handler = simulation.rigidbodies->magnetic->add(handler.rigidbody, stark::EnergyRigidBodyMagnetic::Params()
			.set_magnetic_permeability(mu_neodymium)
			.set_magnetic_material(MagneticMaterial::Permanent)
		);
		// Discretize elements into 2x2x2 dipole set
		simulation.rigidbodies->magnetic->add_magnetic_samples(magnetic_handler, { Eigen::Vector3d::Zero() }, size/2, magnetic_strength*Eigen::Vector3d::UnitX());
		// If magnets are more than 1 side length apart, use explicit solver
		simulation.rigidbodies->magnetic->set_distance_threshold(1.01*size);
	}

	// Add ground surface
	double ground_mass = 1.0;
	const Eigen::Vector3d ground_size(1.0, 1.0, 0.001);
	auto ground = simulation.presets->rigidbodies->add_box("ground",ground_mass,ground_size);
	ground.handler.rigidbody.set_translation({0.0,0.0,-0.001-size/2});
	simulation.rigidbodies->add_constraint_fix(ground.handler.rigidbody);

	// Run
	simulation.run();
}

void magnetic_repulsion_ekw()
{
	// Parameters
	double mass = 0.1; // kg
	double size = 0.1; // m
	double contact_thickness = 0.00005; // m

	// Default example settings
	stark::Settings settings = stark::Settings();
	settings.output.simulation_name = "magnetic_repulsion_ekw";
	settings.output.output_directory = OUTPUT_PATH + "/magnetic_repulsion_ekw";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 1.0;
	settings.debug.symx_check_for_NaNs = true;
	settings.simulation.gravity = { 0.0, 0.0, -9.81 };
	settings.simulation.magnetic_method = stark::MagneticMethod::DipoleMoment;
	settings.simulation.max_time_step_size = 0.001;
	settings.simulation.use_adaptive_time_step = false;
	settings.newton.max_line_search_iterations = 100;
	settings.newton.max_newton_iterations = 1000;
	settings.output.fps = 240;

	stark::Simulation simulation(settings);

	simulation.interactions->contact->set_global_params(
		stark::EnergyFrictionalContact::GlobalParams()
			.set_default_contact_thickness(contact_thickness)
			.set_friction_enabled(true)
			.set_friction_stick_slide_threshold(0.001)
			.set_min_contact_stiffness(1e14)
	);

	// Adding more realistic magnetic parameters
	const double B_r = 1.48; // Tesla - residual flux density of N52 grade magnet
	const double mu_0 = 4*3.14*10e-7; // H/m - permeability of free-space
	const double magnetization = B_r / mu_0;
    double magnetic_strength = magnetization*std::pow(size,3); // A-m^2 - dipole moment

	// Add cube magnets
	int n = 2;
	for (int i = 0; i < n; i++) {
		auto [vertices, triangles, handler] = simulation.presets->rigidbodies->add_box("box", mass, size);
		handler.rigidbody.add_translation({ 2.5*i*size, 0.0, 0.0 });
		auto magnetic_handler = simulation.rigidbodies->magnetic->add(handler.rigidbody, stark::EnergyRigidBodyMagnetic::Params()
			.set_magnetic_permeability(mu_neodymium)
			.set_magnetic_material(MagneticMaterial::Permanent)
		);
		if (i == 1) {magnetic_strength = -magnetic_strength;} // Flip one dipole for repulsion
		// Discretize elements into 2x2x2 dipole set
		simulation.rigidbodies->magnetic->add_magnetic_samples(magnetic_handler, { Eigen::Vector3d::Zero() }, size/2, magnetic_strength*Eigen::Vector3d::UnitX());
		// If magnets are more than 1 side length apart, use explicit solver
		simulation.rigidbodies->magnetic->set_distance_threshold(1.01*size);
	}


	// Add ground
	double ground_mass = 1.0;
	const Eigen::Vector3d ground_size(1.0, 1.0, 0.001);
	auto ground = simulation.presets->rigidbodies->add_box("ground",ground_mass,ground_size);
	ground.handler.rigidbody.set_translation({0.0,0.0,-0.001-size/2});
	simulation.rigidbodies->add_constraint_fix(ground.handler.rigidbody);

	// Run
	simulation.run();
}

// Initial idea was to compare against magnetic failure as maximum timestep increased: 
// I observed that the evaluation time prior to Energy compute was much higher, however
// this implementation started to fail at only dt=0.004, which I suspect is due to numerical issues 
// in how I pushed the two cubes together. I observed that the time span of the 'set_velocity' cube push 
// needed to be proportionally scaled down relative to the timestep scale-up. In future attempts I would dig 
// deeper into how to apply forces to the overall non-magnetic rigid bodies at a simulation level, but
// ultimately, I left this comparison out as it was unfair in that the failure was not controlled to the time step
// variable and was a result of my force implementation.
void nonmagnetic_attraction()
{
	// Parameters
	double mass = 0.1;
	double size = 0.1;
	double contact_thickness = 0.00005;

	stark::Settings settings;
	settings.output.simulation_name = "nonmagnetic_attraction";
	settings.output.output_directory = OUTPUT_PATH + "/nonmagnetic_attraction";
	settings.output.codegen_directory = COMPILE_PATH;
	settings.execution.end_simulation_time = 1.0;
	settings.debug.symx_check_for_NaNs = true;
	settings.simulation.gravity = { 0.0, 0.0, -9.81 };
	settings.simulation.max_time_step_size = 0.004;
	settings.simulation.use_adaptive_time_step = false;
	settings.newton.max_line_search_iterations = 100;
	settings.newton.max_newton_iterations = 1000;
	settings.output.fps = 240;

	stark::Simulation simulation(settings);

	// Contact settings
	simulation.interactions->contact->set_global_params(
		stark::EnergyFrictionalContact::GlobalParams()
			.set_default_contact_thickness(contact_thickness)
			.set_friction_enabled(false)
			.set_min_contact_stiffness(1e14)
	);

	// Add two non-magnetic cubes
	auto [v0, t0, h0] = simulation.presets->rigidbodies->add_box("cube_0", mass, size);
	auto [v1, t1, h1] = simulation.presets->rigidbodies->add_box("cube_1", mass, size);

	// Initial positions (separated along x)
	h0.rigidbody.set_translation({-0.15, 0.0, 0.0});
	h1.rigidbody.set_translation({0.15, 0.0, 0.0});

	// Push the boxes together
	double push_start = 0.0;
	double push_end = 0.5; // First half of sim

	simulation.add_time_event(
		push_start, push_end,
		[&](double t) {
			// linear blend velocity for numerical stability
			double alpha = (t - push_start) / (push_end - push_start);
			h0.rigidbody.set_velocity({1.5 - alpha*1.5, 0.0, 0.0});
			h1.rigidbody.set_velocity({-1.5 +alpha*1.5, 0.0, 0.0});
		}
	);


	// Add ground surface
	double ground_mass = 1.0;
	const Eigen::Vector3d ground_size(1.0, 1.0, 0.001);
	auto ground = simulation.presets->rigidbodies->add_box("ground", ground_mass, ground_size);
	ground.handler.rigidbody.set_translation({ 0.0, 0.0, -0.06 });
	simulation.rigidbodies->add_constraint_fix(ground.handler.rigidbody);

	// Run
	simulation.run();
}


int main()
{
    // Scenes run consecutively.
	magnetic_attraction_ekw();
	magnetic_repulsion_ekw();
	// nonmagnetic_attraction();
}
