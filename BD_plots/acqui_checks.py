
from math import *
from numpy.linalg import norm
from kernel_checks import *
L=6.0
M=20

def local_penalisation(v, busy_samples):
    if len(observations)==0:
        return 1.0
    penalty = 1.0
    print("local penalty:")
    for i in range(len(busy_samples)):
        x = busy_samples[i]
        d = norm(v - x)
        print("v =" , v)
        print("x =", x)
        print("distance =" ,d)
        mu_x = mu(priors[i], x, samples, np.array(observations), np.array(queried_priors),K_inv)
        sigma_x=sigma(x,samples,K_inv)
        z = 1.0 / (sqrt(2 * sigma_x)) * (L * d - M + mu_x[0])
        phi = 0.5 * erfc(-z)
        print("z " +str(z))
        print("phi "+str(phi))
        penalty *= phi
    return penalty


def get_gradnorm(v):

dkn = dKn(v)# Dxt
Eigen::MatrixXd
Kn = _model.get_inv_kernel(); // t
observations_matrix(); // tx1
Eigen::VectorXd
grad = dkn * K_inv * obs;

double
gradnorm = grad.norm();
std::cout << "grad " << grad << std::endl;
std::cout << "gradnorm " << gradnorm << std::endl;
// now
check
finite
difference
method

Eigen::VectorXd
grad_fd = Eigen::VectorXd(dim_in);
double
h = 1e-8;
double
performance = get_performance(v);
for (size_t i=0; i < dim_in();
++i)
{
    Eigen:: VectorXd
v_h = v;
v_h(i) = h;
grad_fd(i) = (get_performance(v_h) - performance) / h;
}

double
gradnorm_finite_diff = grad_fd.norm();
std::cout << "gradnorm  " << gradnorm << std::endl;
std::cout << "gradnorm fd " << gradnorm_finite_diff << std::endl;
return gradnorm;
}

if __name__ == "__main__":
    bds,priors=load_archive()
    remaining_indexes=list(range(len(bds)))
    busy_samples=[]
    samples = []
    observations = []
    queried_priors = []
    noises = []
    K_inv = None

    while True:

        max_ind = None
        max_acq = -float("inf")
        for i in range(len(remaining_indexes)):
            x = bds[i]
            mu_x = mu(priors[i], x, samples, np.array(observations), np.array(queried_priors),K_inv)
            pen = local_penalisation(x,busy_samples)
            print("mu =" + str(mu_x))
            print("update effect = ", M - priors[i])  # inverse relation between update effect size and noise
            if M > max_acq:
                max_acq = M
                max_ind = i

        j = remaining_indexes[max_ind]  # get the corresponding index
        x = bds[j]
        prior = priors[j]
        busy_samples.append(x)
        samples.append(x)
        observations.append(np.random.randint(0, 3))  # low values (0-2) give negative means
        queried_priors.append(prior)
        noises.append(0)  # only extreme noise seems to avoid negative values
        # noises.append(observations[-1] * observations[-1]) # large/variable noise is not cause of negative values
        # update the kernel matrix
        noise_mat = np.array(noises) * np.identity(len(samples))
        Kn = K(samples)
        Kn = Kn + noise_mat
        K_inv = np.linalg.inv(Kn)

