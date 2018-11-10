function Ro = callOrthogonalization(R)
	err = max(max(abs(((R') * R)-eye(3))));
  Ro = R;
  while err > 1.0e-04
    ip = [Ro(:,2)'*Ro(:,3), Ro(:,3)'*Ro(:,1), Ro(:,1)'*Ro(:,2)];
		[m, id] = min(abs(ip));
		t = (cross(Ro(:,mod(id,3)+1), Ro(:,mod(id+1,3)+1)));
		Ro(:,id) = t/norm(t);
		err = max(max(abs(((Ro')*Ro)-eye(3))));
  endwhile
endfunction
