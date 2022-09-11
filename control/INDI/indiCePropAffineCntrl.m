function U = indiCePropAffineCntrl( cep, u ) %#codegen

U = ( sqrt(cep.kt^4+4*cep.d*cep.ri*cep.vb*cep.kt*u) - cep.kt^2 ).^2 ./ ...
    ( sqrt(cep.kt^4+4*cep.d*cep.ri*cep.vb*cep.kt) - cep.kt^2 ).^2;

end