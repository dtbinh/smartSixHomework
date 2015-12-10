function [ Ja ] = pseudoInverseJacobian( q, EulerAngles )

       Ja = pinv(jacobianoAnalitico( q, EulerAngles ))

end

