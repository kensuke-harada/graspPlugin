function R = rotFromRpy(r,p,y)

cr = cos(r);
sr = sin(r);
cp = cos(p);
sp = sin(p);
cy = cos(y);
sy = sin(y);

R = [ cp*cy, sr*sp*cy - cr*sy, cr*sp*cy + sr*sy;
	cp*sy, sr*sp*sy + cr*cy, cr*sp*sy - sr*cy;
             -sp  , sr*cp           , cr*cp];
