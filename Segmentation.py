# some basic setup:
# setup detectron2 logger
import detectron2
from detectron2.utils.logger import setup_logger

setup_logger()

# import some common libraries
import numpy as np
import os, cv2, random
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog
from detectron2.data.datasets import register_coco_instances
from detectron2.engine import DefaultTrainer

print("Environment Ready - Segmentation Model")

def prepare_data():
    # Prepare the dataset
    register_coco_instances("train_dataset", {}, "/home/arl1/Tohar/Data/train_dataset.json", "/home/arl1/Tohar/Data/")
    register_coco_instances("val_dataset", {}, "/home/arl1/Tohar/Data/val_dataset.json", "/home/arl1/Tohar/Data/")
    register_coco_instances("test_dataset", {}, "/home/arl1/Tohar/Data/test_dataset.json", "/home/arl1/Tohar/Data/")

def define_config():
    cfg = get_cfg()
    cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
    cfg.DATASETS.TRAIN = ("train_dataset",)
    cfg.DATASETS.TEST = ("val_dataset",)
    cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")
    cfg.MODEL.BACKBONE.FREEZE_AT = 2
    cfg.SOLVER.BASE_LR = 0.01
    cfg.SOLVER.MAX_ITER = 2700
    cfg.SOLVER.IMS_PER_BATCH = 2  # the number of training images per step/iteration
    cfg.MODEL.ROI_HEADS.NUM_CLASSES = 1  # only has one class (branches)
    return cfg

def training():
    # Train
    cfg = define_config()
    os.makedirs(cfg.OUTPUT_DIR, exist_ok=True)
    trainer = DefaultTrainer(cfg)
    trainer.resume_or_load(resume=False)
    trainer.train()
    return cfg


def model(im):
    # im = cv2.imread('/content/drive/MyDrive/Colab Notebooks/tesis/all_images/29.9_16_11.png')
    cfg = define_config()
    cfg.MODEL.WEIGHTS = os.path.join(cfg.OUTPUT_DIR, "model_final.pth")
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.8
    predictor = DefaultPredictor(cfg)
    outputs = predictor(im)
    v = Visualizer(im[:, :, ::-1], MetadataCatalog.get(cfg.DATASETS.TRAIN[0]), scale=1.5)
    out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
    cv2.imshow('', out.get_image()[:, :, ::-1])
    cv2.imwrite("/home/arl1/Tohar/Outputs/output_color.png", out.get_image()[:, :, ::-1])
    mask_array = outputs['instances'].to("cpu").pred_masks.numpy()
    num_instances = mask_array.shape[0]
    mask_array = np.moveaxis(mask_array, 0, -1)
    mask_array_instance = []
    output = np.zeros_like(im)  # black
    for i in range(num_instances):
        mask_array_instance.append(mask_array[:, :, i:(i + 1)])
        output = np.where(mask_array_instance[i] == True, 255, output)
    output_binary = output[:, :, 0]
    # cv2.imshow('', output_binary)
    cv2.imwrite("/home/arl1/Tohar/Outputs/output.png", output_binary)

