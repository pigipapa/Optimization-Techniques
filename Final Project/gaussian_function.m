function [result] = gaussian_function(u1, u2, c1, c2, sigma1, sigma2, d)

    power = ((u1-c1)^2/(2*sigma1^2)) + ((u2-c2)^2/(2*sigma2^2));
    
    result = d*exp(-power);

end