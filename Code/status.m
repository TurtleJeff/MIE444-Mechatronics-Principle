function type = status(f, rf, rb, lf, lb, b)
r = (rb+rf)/2;
l = (lb+lf)/2;

%type0
if f>19.5 && r>19.5 && l>19.5 && b>19.5
    type=0;
    
    %type1
elseif f<19.5 && r>19.5 && l>19.5 && b>19.5
    type=1;
elseif f>19.5 && r<19.5 && l>19.5 && b>19.5
    type=1;
elseif f>19.5 && r>19.5 && l<19.5 && b>19.5
    type=1;
elseif f>19.5 && r>19.5 && l>19.5 && b<19.5
    type=1;
    
    %type2
elseif f<19.5 && r<19.5 && l>19.5 && b>19.5
    type=2;
elseif f<19.5 && r>19.5 && l<19.5 && b>19.5
    type=2;
elseif f>19.5 && r<19.5 && l>19.5 && b<19.5
    type=2;
elseif f>19.5 && r>19.5 && l<19.5 && b<19.5
    type=2;
    
    %type5
elseif f>19.5 && r<19.5 && l<19.5 && b>19.5
    type=5;
elseif f<19.5 && r>19.5 && l>19.5 && b<19.5
    type=5;
    
    %type3
elseif f>19.5 && r<19.5 && l<19.5 && b<19.5
    type=3;
elseif f<19.5 && r>19.5 && l<19.5 && b<19.5
    type=3;
elseif f<19.5 && r<19.5 && l>19.5 && b<19.5
    type=3;
elseif f<19.5 && r<19.5 && l<19.5 && b>19.5
    type=3;
end
end