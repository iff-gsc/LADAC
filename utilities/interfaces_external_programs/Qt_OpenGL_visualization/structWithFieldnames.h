#include <QString>
#include <QVector>
#include <qmath.h>
#include <QDataStream>
#include <cstdlib>

typedef float typeudp;


class StructWithFieldnames {
public:
    void setFromStream(QDataStream & stream);
    QVector<QString> getSubStructNames();
    QVector<QString> getSubDataNames();
    QVector<QString> getSubNames();
    StructWithFieldnames getSubStruct(QString structName);
    QVector<double> getSubData(QString dataName);

    // to do
    //StructWithFieldnames getSubStruct(QVector<QString> strucNames);
    // QVector<double> getSubData(QVector<QString> dataNames);

private:
    // variables
    QVector<QString> m_varName;
    QVector<int> m_varIndex;
    QVector<int> m_level;
    QVector<bool> m_isStruct;
    QVector<QVector<int>> m_assignmentIndex;
    QVector<double> m_data;

    // functions
    int getNumberOfFurtherLevels();
    int getNumberOfFurtherLevels(QVector<QString> structNames);

    int getIndexFromAssignmentIndex(int assigmentIndex);
    StructWithFieldnames getNextLayerInfo(int varIndex);
    StructWithFieldnames getNextLayerInfoRecursive(int varIndex);
    void sortIndices();

    /*
    // to do
    QVector<int> getAllNestedStructIndices();
    QVector<int> getAllNestedStructIndices(QString structName);
    QVector<int> getAllNestedStructIndices(QVector<QString> structNames);

    QVector<QString> getAllNestedStructNames();
    QVector<QString> getAllNestedStructNames(QString structNames);
    QVector<QString> getAllNestedStructNames(QVector<QString> structNames);

    QVector<QString> getAllNamesAtLevel(int level); // not important
    QVector<QString> getAllStructNamesAtLevel(int level); // not important
    QVector<QString> getAllDataNamesAtLevel(int level); // not important
    */

};


