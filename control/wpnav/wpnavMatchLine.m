function [p_match,t,d] = wpnavMatchLine(p1,p2,p)

b = p2-p1;

denom = dot(b,b);
if denom < 1
    denom = 1;
end

t = divideFinite( dot(p-p1,b), denom );

p_match = wpnavLineGetPos(p1,p2,t);
d = norm( p_match - p, 2);

end