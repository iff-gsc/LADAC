#include "structWithFieldnames.h"


QVector<QString> StructWithFieldnames::getSubStructNames() {
    QVector<QString> subStructNames;
    for (int i=0; i<m_varName.length(); i++) {
        if (m_level[i] == 1 && m_isStruct[i]) {
            subStructNames.append(m_varName[i]);
        }
    }
    return subStructNames;
}

QVector<QString> StructWithFieldnames::getSubDataNames() {
    QVector<QString> subDataNames;
    for (int i=0; i<m_varName.length(); i++) {
        if (m_level[i] == 1 && !m_isStruct[i]) {
            subDataNames.append(m_varName[i]);
        }
    }
    return subDataNames;
}

QVector<QString> StructWithFieldnames::getSubNames() {
    QVector<QString> subNames;
    for (int i=0; i<m_varName.length(); i++) {
        if (m_level[i] == 1) {
            subNames.append(m_varName[i]);
        }
    }
    return subNames;
}

StructWithFieldnames StructWithFieldnames::getSubStruct(QString structName) {
    StructWithFieldnames subStruct;
    int varIndex = -1;
    for (int i=0; i<m_varName.length(); i++) {
        if (m_varName[i] == structName && m_isStruct[i] && m_level[i] == 1) {
            varIndex = m_varIndex[i];
            break;
        }
    }
    StructWithFieldnames subStructInfoRec = getNextLayerInfoRecursive(varIndex);
    for (int i=0; i<subStructInfoRec.m_level.length(); i++) {
        subStructInfoRec.m_level[i]--;
    }
    subStructInfoRec.m_data = m_data;
    subStructInfoRec.sortIndices();
    return subStructInfoRec;
}

QVector<double> StructWithFieldnames::getSubData(QString structName) {
    QVector<double> subData;
    int varIndex = -1;
    for (int i=0; i<m_varName.length(); i++) {
        if (m_varName[i] == structName && !m_isStruct[i] && m_level[i] == 1) {
            varIndex = m_varIndex[i];
            break;
        }
    }
    if (varIndex>=0) {
        QVector<int> assignmentIndex = m_assignmentIndex[varIndex];
        for (int i=0; i<assignmentIndex.length(); i++) {
            subData.append(m_data[assignmentIndex[i]]);
        }
    }
    return subData;
}


int StructWithFieldnames::getNumberOfFurtherLevels() {
    int* numberOfFurtherLevels = std::max_element(&m_level[0],&m_level[0]+m_level.size());
    return *numberOfFurtherLevels - 1;
}


// Ugly code, but it works.
void StructWithFieldnames::setFromStream(QDataStream & stream) {

    // init
    int identifier;
    int currentIdentifier;
    int varIndex = 0;
    QVector<int> dataIndex;
    int16_t numberOfDataPoints = 0;
    typeudp numberOfDataPointsD = 0;
    QString field_name;
    QVector<typeudp> field_val;
    QVector<int> assignmentIndex;
    QVector<int> init_asgnmt_idx = {-1};
    typeudp val = 0;
    bool new_struct = false;
    bool read_name = false;
    int read_name_idx;
    bool read_data = false;
    int field_name_length;

    // the very first element of the stream is the length
    stream >> numberOfDataPointsD;
    numberOfDataPoints = (int) round(numberOfDataPointsD);

    // decode the stream from the first element to the last
    for (int i=0; i<numberOfDataPoints; i++) {
        stream >> val;
        // the first element is the identifier
        if (i==0) {
            identifier = val;
            currentIdentifier = identifier;
        }
        // did we find an identifier?
        if (round(val)>=identifier && round(val)<=currentIdentifier+1) {
            new_struct = true;
            read_data = false;
            dataIndex.clear();
            // did the level increase?
            if (round(val)==currentIdentifier+1) {
                currentIdentifier++;
                // ... then the last element was a struct (exept it was the first one)
                if (m_level.size()>0) {
                    m_isStruct.append(true);
                }
            }
            // did the level decrease?
            else if (round(val)<currentIdentifier) {
                // ... then the last element was no struct
                m_isStruct.append(false);
                int num_finished_structs = currentIdentifier - round(val);
                currentIdentifier = round(val);
                // depending on how many levels it has decreased, we now can assign the indices of all structs that are now entirely read
                for (int l=0; l<num_finished_structs; l++) {
                    assignmentIndex.clear();
                    for (int j=m_varIndex.last(); j>=0; j--) {
                        if (m_level[j]==m_level.last()-l) {
                            assignmentIndex.prepend(j);
                        }
                        if (m_level[j]<m_level.last()-l) {
                            for (int k=0; k<assignmentIndex.length(); k++) {
                                // overwrite first element (from initialization) or append vector
                                if (m_assignmentIndex[j].first()==init_asgnmt_idx.first()) {
                                    m_assignmentIndex[j].first() = assignmentIndex[k];
                                }
                                else {
                                    m_assignmentIndex[j].append(assignmentIndex[k]);
                                }
                            }
                            break;
                        }
                    }
                }
            }
            // if the identifier hasn't changed, it is no struct
            else {
                // ... exept for the very first element
                if (m_level.size()>0) {
                    m_isStruct.append(false);
                }
            }
        }
        // no identifier was detected, so we read the data (there are 3 states)
        else {
            if (read_data) {
                // 3rd state: read data of variable
                // set data to the member variable
                m_data.append((double) val);
                // overwrite first element (from initialization) or append vector
                if (m_assignmentIndex[m_varIndex.last()].first()==init_asgnmt_idx.first()) {
                    m_assignmentIndex[m_varIndex.last()].first() = m_data.length()-1;
                }
                else {
                    m_assignmentIndex[m_varIndex.last()].append(m_data.length()-1);
                }
                // the very last variable can not be a struct
                if (i==numberOfDataPoints-1) {
                    m_isStruct.append(false);
                    // depending on how many levels it has decreased, we now can assign the indices of all structs that are now entirely read
                    // (copy paste)
                    int num_finished_structs = m_level.last()-1;
                    for (int l=0; l<num_finished_structs; l++) {
                        assignmentIndex.clear();
                        for (int j=m_varIndex.last(); j>=0; j--) {
                            if (m_level[j]==m_level.last()-l) {
                                assignmentIndex.prepend(j);
                            }
                            if (m_level[j]<m_level.last()-l) {
                                for (int k=0; k<assignmentIndex.length(); k++) {
                                    // overwrite first element (from initialization) or append vector
                                    if (m_assignmentIndex[j].first()==init_asgnmt_idx.first()) {
                                        m_assignmentIndex[j].first() = assignmentIndex[k];
                                    }
                                    else {
                                        m_assignmentIndex[j].append(assignmentIndex[k]);
                                    }
                                }
                                break;
                            }
                        }
                    }
                }
            }
            else if (read_name) {
                // 2nd state: read variable name
                field_name += (int) round(val);
                read_name_idx++;
                if (read_name_idx==field_name_length) {
                    // change state to: read data
                    read_name = false;
                    read_data = true;
                    // set the member variables that are known here
                    m_varName.append(field_name);
                    m_level.append(currentIdentifier-identifier+1);
                    m_varIndex.append(varIndex++);
                    m_assignmentIndex.append(init_asgnmt_idx); // init only
                }
            }
            else if (new_struct) {
                // 1st state: new struct detected - get length of its name
                field_name_length = (int) round(val);
                field_name = "";
                field_val = {};
                new_struct = false;
                read_name = true;
                read_name_idx = 0;
            }
        }
    }
}

StructWithFieldnames StructWithFieldnames::getNextLayerInfo(int varIndex) {
    StructWithFieldnames nextLayer;
    int index = getIndexFromAssignmentIndex(varIndex);
    if (m_isStruct[index]) {
        for (int i=0; i<m_assignmentIndex[varIndex].length(); i++) {
            if (m_assignmentIndex[varIndex][i]>=0) {
                index = getIndexFromAssignmentIndex(m_assignmentIndex[varIndex][i]);
                nextLayer.m_varName.append(m_varName[index]);
                nextLayer.m_level.append(m_level[index]);
                nextLayer.m_isStruct.append(m_isStruct[index]);
                nextLayer.m_varIndex.append(m_varIndex[index]);
                nextLayer.m_assignmentIndex.append(m_assignmentIndex[index]);
            }
        }
    }
    return nextLayer;
}

int StructWithFieldnames::getIndexFromAssignmentIndex(int assigmentIndex) {
    int index = -1;
    for (int i=0; i<m_varName.length(); i++) {
        if (m_varIndex[i]==assigmentIndex) {
            index = i;
            break;
        }
    }
    return index;
}

StructWithFieldnames StructWithFieldnames::getNextLayerInfoRecursive(int varIndex) {
    StructWithFieldnames subStructInfo = getNextLayerInfo(varIndex);
    StructWithFieldnames subStructInfoRec;
    for (int i=0; i<subStructInfo.m_varName.length(); i++) {
        subStructInfoRec.m_varName.append(subStructInfo.m_varName[i]);
        subStructInfoRec.m_level.append(subStructInfo.m_level[i]);
        subStructInfoRec.m_isStruct.append(subStructInfo.m_isStruct[i]);
        subStructInfoRec.m_varIndex.append(subStructInfo.m_varIndex[i]);
        subStructInfoRec.m_assignmentIndex.append(subStructInfo.m_assignmentIndex[i]);
        if (subStructInfo.m_isStruct[i]) {
            StructWithFieldnames subStructInfo2 = getNextLayerInfoRecursive(subStructInfo.m_varIndex[i]);
            for (int j=0; j<subStructInfo2.m_varName.length(); j++) {
                subStructInfoRec.m_varName.append(subStructInfo2.m_varName[j]);
                subStructInfoRec.m_level.append(subStructInfo2.m_level[j]);
                subStructInfoRec.m_isStruct.append(subStructInfo2.m_isStruct[j]);
                subStructInfoRec.m_varIndex.append(subStructInfo2.m_varIndex[j]);
                subStructInfoRec.m_assignmentIndex.append(subStructInfo2.m_assignmentIndex[j]);
            }
        }
    }
    return subStructInfoRec;
}

void StructWithFieldnames::sortIndices() {
    StructWithFieldnames copy = *this;
    m_data.clear();
    int dataIndex = 0;
    for (int i=0; i<m_varName.length(); i++) {
        if (m_varIndex[i]!=i) {
            m_varIndex[i] = i;
            for (int j=0; j<m_varName.length(); j++) {
                if (m_isStruct[j]) {
                    for (int k=0; k<m_assignmentIndex[j].length(); k++) {
                        if (copy.m_assignmentIndex[j][k]==copy.m_varIndex[i]) {
                            m_assignmentIndex[j][k] = i;
                        }
                    }
                }
            }
        }
        if (!m_isStruct[i]) {
            for (int l=0; l<m_assignmentIndex[i].length(); l++) {
                m_assignmentIndex[i][l] = dataIndex++;
                m_data.append(copy.m_data[copy.m_assignmentIndex[i][l]]);
            }
        }
    }
}
