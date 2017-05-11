double pidController(double error) {

	double pid_output = 0;

	// Calculate p term
	p_term = P * error;

	// Calculate i term with clamping (anti windup)
	if (clamp(preSat, error)) i_term = i_term + 0;
	else i_term = i_term + (error*SAMPLE_TIME);
	i_term = i_term*I;

	// Calculate d term with filtered derivative
	d_term = (D * error - d_filter) * N;

	// Summarize the p,i,d terms
	preSat = p_term + i_term + d_term;

	// Saturate output for max / min 255 / -255
	if (preSat > 255) pid_output = 255;
	else if (preSat < -255) pid_output = -255;
	else pid_output = preSat;

	d_filter = d_filter + d_term*SAMPLE_TIME;

	return pid_output;
}

bool clamp(double preSat, double preIntegrator) {

	double deadZone_out = 0;
	int signDeltaU = 0;
	int signPreIntegrator = 0;

	// Check if the presat value is inside the deadzone
	if ((preSat <= UPPER_SATURATION_LIMIT) & (preSat >= LOWER_SATURATION_LIMIT)) deadZone_out = 0;
	else {
		if ((preSat > UPPER_SATURATION_LIMIT)) deadZone_out = preSat - UPPER_SATURATION_LIMIT;
		else if ((preSat < LOWER_SATURATION_LIMIT)) deadZone_out = preSat - LOWER_SATURATION_LIMIT;
	}

	// Calculate sign delta u
	if (deadZone_out > 0) signDeltaU = 1;
	else if (deadZone_out == 0) signDeltaU = 0;
	else if (deadZone_out < 0) signDeltaU = -1;

	// Calculate sign pre integrator
	if (preIntegrator > 0) signPreIntegrator = 1;
	else if (preIntegrator == 0) signPreIntegrator = 0;
	else if (preIntegrator < 0) signPreIntegrator = -1;

	// Return true if both signs equal and the preSat outside the dead zone
	if ((signDeltaU == signPreIntegrator) & (deadZone_out != 0)) return true;
}
