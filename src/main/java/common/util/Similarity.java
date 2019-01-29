package common.util;

public class Similarity {
    public static boolean isMatch(int[] features1, int[] features2, double threshold) {
        return calcSimilarity(features1, features2) >= threshold;
    }

    public static double calcSimilarity(int[] features1, int[] features2) {
        return crossProduct(features1, features2)/features1.length;
    }

    private static double crossProduct(int[] features1, int[] features2) {
        double crossProduct = 0;
        if (features1.length == features2.length) {
            for (int i=0; i < features1.length; i++) {
                crossProduct += features1[i] * features2[i];
            }
        }
        return crossProduct;
    }
}