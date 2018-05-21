
import cozmo
import numpy as np
import asyncio
import imgclassification

async def cozmoBehavior(robot: cozmo.robot.Robot):
    global img_clf

    robot.camera.image_stream_enabled = True
    await robot.set_lift_height(0).wait_for_completed()
    await robot.set_head_angle(cozmo.util.degrees(10)).wait_for_completed()

    while True:
        cameracapture = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=5)
        #(test_raw, test_labels) = img_clf.load_data_from_folder('./test/')
        #test_data = img_clf.extract_image_features(test_raw)

        img = np.asarray(cameracapture.image)

        features = img_clf.extract_image_features(np.array([img]))
        [label] = img_clf.predict_labels(features)
        #print(str(label))

        await robot.say_text(label).wait_for_completed()

if __name__ == "__main__":
    img_clf = imgclassification.ImageClassifier()
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    train_data = img_clf.extract_image_features(train_raw)
    img_clf.train_classifier(train_data, train_labels)
    #cozmoBehavior()
    cozmo.run_program(cozmoBehavior, use_viewer=False,
                      force_viewer_on_top=True)

