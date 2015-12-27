// camera location lX, lY, lZ; looking at vX, vY, vZ; +i(forward), -i(backward)

void walk(double &lX, double &lY, double &lZ,
			double &vX, double &vY, double &vZ, int d, double speed){
	// get vector i, j, k
	double rvi = vX - lX;
	double rvj = vY - lY;
	double rvk = vZ - lZ;
	
	double change = 1.0 * speed;
	if(d > 0){
		change = change * (-1);
	}

	double mod = sqrt(rvi*rvi+rvj*rvj+rvk*rvk);

	printf("v - %lf  %lf   %lf\n", vX, vY, vZ);
	printf("l - %lf  %lf   %lf\n", lX, lY, lZ);

	if(mod != 0){
		lX = lX - rvi * 1.0 * change / mod;
		lY = lY - rvj * 1.0 * change / mod;
		lZ = lZ - rvk * 1.0 * change / mod;
		
		vX = vX - rvi * 1.0 * change / mod;
		vY = vY - rvj * 1.0 * change / mod;
		vZ = vZ - rvk * 1.0 * change / mod;
	}
}

// uX, uY, uZ up direction
void straffing(double &lX, double &lY, double &lZ, 
				double &vX, double &vY, double &vZ, 
				double uX, double uY, double uZ, int d, double speed){
	
	// get vector i, j, k
	double vi = vX - lX;
	double vj = vY - lY;
	double vk = vZ - lZ;
	
	// (view vector) X (up vector) ; cross product; resultant vector
	double rvi = vj*uZ - vk*uY;
	double rvj = vk*uX - vi*uZ;
	double rvk = vi*uY - vj*uX;	
	double mod = sqrt(rvi*rvi+rvj*rvj+rvk*rvk);
	
	double change = 1.0 * speed;
	if(d > 0){
		change = change * (-1);
	}
	
	printf("v - %lf  %lf   %lf\n", vX, vY, vZ);
	printf("l - %lf  %lf   %lf\n", lX, lY, lZ);
	if(mod != 0){
		lX = lX - rvi * 1.0 * change / mod;
		lY = lY - rvj * 1.0 * change / mod;
		lZ = lZ - rvk * 1.0 * change / mod;
		
		vX = vX - rvi * 1.0 * change / mod;
		vY = vY - rvj * 1.0 * change / mod;
		vZ = vZ - rvk * 1.0 * change / mod;
	}
}

// movement will be along the updirection vector; uX, uY, uZ
void fly(double &lX, double &lY, double &lZ, 
				double &vX, double &vY, double &vZ, 
				double uX, double uY, double uZ, int d, double speed){
	
	// up vector is the resultant vector
	double rvi = uX;
	double rvj = uY;
	double rvk = uZ;	
	double mod = sqrt(rvi*rvi+rvj*rvj+rvk*rvk);
	
	double change = 1.0 * speed;
	if(d < 0){
		change = change * (-1);
	}	
		
	printf("v - %lf  %lf   %lf\n", vX, vY, vZ);
	printf("l - %lf  %lf   %lf\n", lX, lY, lZ);
	if(mod != 0){
		lX = lX + rvi * 1.0 * change / mod;
		lY = lY + rvj * 1.0 * change / mod;
		lZ = lZ + rvk * 1.0 * change / mod;
		
		vX = vX + rvi * 1.0 * change / mod;
		vY = vY + rvj * 1.0 * change / mod;
		vZ = vZ + rvk * 1.0 * change / mod;
	}	
}

// rodrigues formula; vrot = vcost + (k X v)sint + k(k.V)(1 - cost)


void yaw(double &lX, double &lY, double &lZ, 
				double &vX, double &vY, double &vZ, 
				double uX, double uY, double uZ, int d, double speed){
	
	// get vector i, j, k
	double vi = vX - lX;
	double vj = vY - lY;
	double vk = vZ - lZ;
	double mod = sqrt(uX*uX+uY*uY+uZ*uZ);
	uX = uX * 1.0 / mod;
	uY = uY * 1.0 / mod;
	uZ = uZ * 1.0 / mod;
	
	double theta = 0.05 * d * speed; // 10 degree = .17 radian
	double cost = cos(theta);	
	double sint = sin(theta);
	
	printf("%lf  %lf   %lf\n", vX, vY, vZ);
	vX = lX + (vi * cost) + ((uY * vk - uZ * vj) * sint ) + ((uX * vi * uX) * (1 - cost));
	vY = lY + (vj * cost) + ((uZ * vi - uX * vk) * sint ) + ((uY * vj * uY) * (1 - cost));
	vZ = lZ + (vk * cost) + ((uX * vj - uY * vi) * sint ) + ((uZ * vk * uZ) * (1 - cost));
}

void pitch(double &lX, double &lY, double &lZ, 
				double &vX, double &vY, double &vZ, 
				double &uX, double &uY, double &uZ, int d, double speed){
	
	// get vector i, j, k
	double vi = vX - lX;
	double vj = vY - lY;
	double vk = vZ - lZ;
	
	double cx = uY * vk - uZ * vj;
	double cy = uZ * vi - uX * vk;
	double cz = uX * vj - uY * vi;
	double mod = sqrt(cx*cx+cy*cy+cz*cz);
	cx = cx / mod;
	cy = cy / mod;
	cz = cz / mod;
	
	double theta = 0.05 * d * speed; // 10 degree = .17 radian
	double cost = cos(theta);	
	double sint = sin(theta);
	
	printf("v - %lf  %lf   %lf\n", vX, vY, vZ);
	vX = lX + (vi * cost) + ((cy * vk - cz * vj) * sint ) + ((cx * vi * cx) * (1 - cost));
	vY = lY + (vj * cost) + ((cz * vi - cx * vk) * sint ) + ((cy * vj * cy) * (1 - cost));
	vZ = lZ + (vk * cost) + ((cx * vj - cy * vi) * sint ) + ((cz * vi * cz) * (1 - cost));
	
	printf("u - %lf  %lf   %lf\n", uX, uY, uZ);
	uX = (uX * cost) + ((cy * uZ - cz * uY) * sint ) + ((cx * uX * cx) * (1 - cost));
	uY = (uY * cost) + ((cz * uX - cx * uZ) * sint ) + ((cy * uY * cy) * (1 - cost));
	uZ = (uZ * cost) + ((cx * uY - cy * uX) * sint ) + ((cz * uZ * cz) * (1 - cost));	
}

void rotate(double &lX, double &lY, double &lZ, 
				double &vX, double &vY, double &vZ, 
				double &uX, double &uY, double &uZ, int d, double speed){
	
	// get vector i, j, k
	double vi = vX - lX;
	double vj = vY - lY;
	double vk = vZ - lZ;
	double mod = sqrt(vi*vi+vj*vj+vk*vk);
	vi = vi * 1.0 / mod;
	vj = vj * 1.0 / mod;
	vk = vk * 1.0 / mod;
	
	double theta = 0.05 * d * speed; // 10 degree = .17 radian
	double cost = cos(theta);	
	double sint = sin(theta);

	printf("u - %lf  %lf   %lf\n", uX, uY, uZ);
	uX = (uX * cost) + ((vj * uZ - vk * uY) * sint ) + ((vi * uX * vi) * (1 - cost));
	uY = (uY * cost) + ((vk * uX - vi * uZ) * sint ) + ((vj * uY * vj) * (1 - cost));
	uZ = (uZ * cost) + ((vi * uY - vj * uX) * sint ) + ((vk * uZ * vk) * (1 - cost));	
}

// extra functionality for testing
void rotateCamera(double &lX, double &lY, double &lZ, 
				double &vX, double &vY, double &vZ, 
				double uX, double uY, double uZ, int d, double speed){
	
	// get vector i, j, k
	double vi = lX - vX;
	double vj = lY - vY;
	double vk = lZ - vZ;
	double mod = sqrt(uX*uX+uY*uY+uZ*uZ);
	uX = uX * 1.0 / mod;
	uY = uY * 1.0 / mod;
	uZ = uZ * 1.0 / mod;
	
	double theta = 0.05 * d * speed; // 10 degree = .17 radian
	double cost = cos(theta);	
	double sint = sin(theta);
	
	printf("%lf  %lf   %lf\n", lX, lY, lZ);
	lX = vX + (vi * cost) + ((uY * vk - uZ * vj) * sint ) + ((uX * vi * uX) * (1 - cost));
	lY = vY + (vj * cost) + ((uZ * vi - uX * vk) * sint ) + ((uY * vj * uY) * (1 - cost));
	lZ = vZ + (vk * cost) + ((uX * vj - uY * vi) * sint ) + ((uZ * vk * uZ) * (1 - cost));
}

void rotateCamera2(double &lX, double &lY, double &lZ, 
				double &vX, double &vY, double &vZ, 
				double &uX, double &uY, double &uZ, int d, double speed){
	
	// get vector i, j, k
	double vi = lX - vX;
	double vj = lY - vY;
	double vk = lZ - vZ;
	
	double cx = uY * vk - uZ * vj;
	double cy = uZ * vi - uX * vk;
	double cz = uX * vj - uY * vi;
	double mod = sqrt(cx*cx+cy*cy+cz*cz);
	cx = cx / mod;
	cy = cy / mod;
	cz = cz / mod;
	
	double theta = 0.05 * d * speed; // 10 degree = .17 radian
	double cost = cos(theta);	
	double sint = sin(theta);
	
	printf("v - %lf  %lf   %lf\n", vX, vY, vZ);
	lX = vX + (vi * cost) + ((cy * vk - cz * vj) * sint ) + ((cx * vi * cx) * (1 - cost));
	lY = vY + (vj * cost) + ((cz * vi - cx * vk) * sint ) + ((cy * vj * cy) * (1 - cost));
	lZ = vZ + (vk * cost) + ((cx * vj - cy * vi) * sint ) + ((cz * vi * cz) * (1 - cost));
	
	printf("u - %lf  %lf   %lf\n", uX, uY, uZ);
	uX = (uX * cost) + ((cy * uZ - cz * uY) * sint ) + ((cx * uX * cx) * (1 - cost));
	uY = (uY * cost) + ((cz * uX - cx * uZ) * sint ) + ((cy * uY * cy) * (1 - cost));
	uZ = (uZ * cost) + ((cx * uY - cy * uX) * sint ) + ((cz * uZ * cz) * (1 - cost));	
}