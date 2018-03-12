import numpy as np
import argparse
from features_extraction import *
from sklearn.externals import joblib
import rospkg

    # For each point, downsample to 200 points
    # generate 28 dimensional features 
def classification(obj_points_all):
    # Loading the classifier
    rospack = rospkg.RosPack()
    classifier_path = rospack.get_path('object_detection')
    classifier = classifier_path + '/scripts/trained_classifier_4classes.pkl'
    classify = joblib.load(classifier)
    obj_types = ['pedestrian', 'car', 'cyclist', 'misc']
    obj_features = np.zeros((len(obj_points_all), NUM_FEATURES))
    for i, obj_points in enumerate(obj_points_all):
        obj_features[i] = feature_extraction(obj_points)
    # Classifying objects
    obj_classified = classify.predict(obj_features)
    obj_classified = obj_classified.astype(np.int32)
#    print('obj_classified: {}'.format([obj_types[i] for i in obj_classified]))
    return obj_types, obj_classified


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('source', nargs='+', help='list of .txt objects for testing')
    parser.add_argument('classifier', help='Please provide the classifer .pkl file')
    args = parser.parse_args()
    
    f = test(args.source, args.classifier)
    print(f)
    return 


if __name__ == '__main__':
    main()
