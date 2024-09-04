import matplotlib.pyplot as plt

# Data from the problem statement
normal_pos = [0.00207, 0.1021, 0.2014, 0.307, 0.4069, 0.5, 0.6045, 0.7068, 0.8042, 0.9056, 1]

x_focal = [3137.679848928077, 3095.184793832082, 3009.864512185836, 2963.233287060973,
           2968.751535559142, 2880.299435296937, 2825.615488417203, 2856.537554895072, 2793.370878971413,
           2770.068413360319, 2731.520854676508]

y_focal = [3135.716047577978, 3091.149295440168, 3007.879045792509, 2958.064336384845,
           2967.011751016502, 2876.086872865471, 2823.911736779182, 2853.555752572541, 2790.991674241141,
           2768.467675888526, 2730.180605403247]

x_principal_point = [963.8956050774203, 967.8326166821957, 977.0077090353396, 963.4703717196917,
                     981.900175762902, 984.3835821258267, 958.6228048096403, 961.2743486287603, 971.3647555958406,
                     990.2797064172353, 939.5584939926101]

y_principal_point = [521.6096572066454, 524.2971499692793, 517.8522384481392, 509.8610087415926,
                     529.0640651478625, 501.3521118742215, 517.2875317256787, 527.8856864146385, 521.2150944829865,
                     506.7801609777343, 528.2908786243755]

k1 = [-0.07176294572986056, -0.1016638390432257, -0.1044357653239255, -0.08293845964279872,
      -0.1339333473188663, -0.1258098614128354, -0.09132915498525751, -0.09296875887312464, -0.0938645993925124,
      -0.1134421193528152, -0.08701403466156693]

k2 = [0.1077828821947737, 0.2530607437710068, 0.1604719285571378, -0.05853846092968927,
      0.5521149695909472, 0.2474632223807354, -0.02945172340374569, 0.01997047167226074, -0.007455381757169214,
      0.1730715368038928, -0.1390048456768244]

# Create subplots
fig, axes = plt.subplots(3, 2, figsize=(12, 10))
fig.suptitle('Intrinsic Parameters of Camera')

# Plotting each graph
axes[0, 0].plot(normal_pos, x_focal, marker='o')
axes[0, 0].set_title('x-focal length vs normal_pos')
axes[0, 0].set_xlabel('normal_pos')
axes[0, 0].set_ylabel('x-focal length')

axes[0, 1].plot(normal_pos, y_focal, marker='o')
axes[0, 1].set_title('y-focal length vs normal_pos')
axes[0, 1].set_xlabel('normal_pos')
axes[0, 1].set_ylabel('y-focal length')

axes[1, 0].plot(normal_pos, x_principal_point, marker='o')
axes[1, 0].set_title('x principal point vs normal_pos')
axes[1, 0].set_xlabel('normal_pos')
axes[1, 0].set_ylabel('x principal point')

axes[1, 1].plot(normal_pos, y_principal_point, marker='o')
axes[1, 1].set_title('y principal point vs normal_pos')
axes[1, 1].set_xlabel('normal_pos')
axes[1, 1].set_ylabel('y principal point')

axes[2, 0].plot(normal_pos, k1, marker='o')
axes[2, 0].set_title('k1 (distortion) vs normal_pos')
axes[2, 0].set_xlabel('normal_pos')
axes[2, 0].set_ylabel('k1 (distortion coefficient)')

axes[2, 1].plot(normal_pos, k2, marker='o')
axes[2, 1].set_title('k2 (distortion) vs normal_pos')
axes[2, 1].set_xlabel('normal_pos')
axes[2, 1].set_ylabel('k2 (distortion coefficient)')

# Adjust layout
plt.tight_layout(rect=[0, 0.03, 1, 0.95])
plt.show()
