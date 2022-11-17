function edgeRemoval( h )
% Gets the handle h to a surf object, removes the stretched triangles causing
% artifacts on the edges of objects
fn = h.FaceNormals;
fn_size = size(fn);
for l = 1:fn_size(1)
    for ll = 1:fn_size(2)
        if fn(l, ll, 1) > fn(l, ll, 3) || fn(l, ll, 2) > fn(l, ll, 3)
            h.CData(l, ll, :) = NaN;
        end
    end
end

