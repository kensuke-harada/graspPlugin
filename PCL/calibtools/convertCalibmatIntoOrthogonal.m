fid_input = fopen('calibmat.txt', 'r');
if fid_input < 0
 exit;
end

s = 1;
in_matrix = 0;
output = "";
while (s = fgets(fid_input)) > 0
	if index(s,"#") != 0
	  output = strcat(output, s);
	  continue;
	endif
	if index(s, "4 4") != 0
		m = eye(4);
		for i=1:4
		  l = fgets(fid_input);
      m(i,:) = (sscanf(l,"%f %f %f %f"))';
		end
    m([1:3],[1:3]) = callOrthogonalization(m([1:3],[1:3]));
    output = strcat(output, "4 4\n");
    for i=1:4
      output = strcat(output, sprintf("%f %f %f %f\n", m(i,1), m(i,2), m(i,3), m(i,4)));
    end
	end
end

fid_output = fopen('calibmat_ortho.txt', 'w');
fputs(fid_output, output);
