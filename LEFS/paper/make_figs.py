import pymupdf, os

doc = pymupdf.open("paper/2606.12406v2.pdf")
os.makedirs("paper/figures", exist_ok=True)

# clip regions in PDF points (page = 612x792)
jobs = [
    (7, (95, 48, 470, 158), "fig06_task_progress.png"),          # Figure 6
    (7, (300, 165, 480, 250), "fig07_validation_loss.png"),      # Figure 7
    (18, (95, 60, 480, 265), "fig11_act_progress.png"),          # Figure 11
]
for pno, rect, name in jobs:
    page = doc[pno]
    pix = page.get_pixmap(dpi=500, clip=pymupdf.Rect(*rect))
    pix.save("paper/figures/" + name)
    print(name, pix.width, pix.height)

# copy the good raster figures with friendly names
import shutil
mapping = {
    "fig_01_p01_5885x2143.png": "fig01_overview.png",
    "fig_02_p04_1735x499.png": "fig02_deployment.png",
    "fig_03_p05_5565x2338.png": "fig03_tasks.png",
    "fig_04_p06_1015x814.png": "fig04_user_study_franka.png",
    "fig_05_p07_2907x1200.png": "fig05_torque_curves.png",
    "fig_06_p14_7012x4010.png": "fig08_dataset_coverage.png",
    "fig_07_p16_852x598.png": "fig09_user_study_piper.png",
    "fig_08_p18_1744x936.png": "fig10_segmentation.png",
    "fig_09_p20_1862x614.png": "fig12_upsample_magnitude.png",
    "fig_10_p22_1332x828.png": "fig13_attention.png",
}
for src, dst in mapping.items():
    shutil.copy("paper/images/" + src, "paper/figures/" + dst)
print("figures ready:", len(os.listdir("paper/figures")))
